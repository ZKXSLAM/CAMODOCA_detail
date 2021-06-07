#include "camodocal/calib/CamOdoCalibration.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <complex>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include "camodocal/EigenUtils.h"
#include "ceres/ceres.h"

#include <boost/thread/mutex.hpp>

namespace camodocal
{

class CameraOdometerError
{
public:
    CameraOdometerError(Eigen::Vector3d r1, Eigen::Vector3d t1,
                        Eigen::Vector3d r2, Eigen::Vector3d t2)
        : m_rvec1(r1), m_tvec1(t1), m_rvec2(r2), m_tvec2(t2)
    {}

    template<typename T>
    bool operator() (const T* const q4x1, const T* const t3x1,
                     const T* const scale, T* residuals) const
    {
        Eigen::Quaternion<T> q(q4x1[0], q4x1[1], q4x1[2], q4x1[3]);
        Eigen::Matrix<T,3,1> t;
        t << t3x1[0], t3x1[1], T(0);

        Eigen::Matrix<T,3,1> r1 = m_rvec1.cast<T>();
        Eigen::Matrix<T,3,1> t1 = m_tvec1.cast<T>();
        Eigen::Matrix<T,3,1> r2 = m_rvec2.cast<T>();
        Eigen::Matrix<T,3,1> t2 = m_tvec2.cast<T>();

        Eigen::Quaternion<T> q1 = AngleAxisToQuaternion<T>(r1);
        Eigen::Quaternion<T> q2 = AngleAxisToQuaternion<T>(r2);

        Eigen::Matrix<T,3,3> R1 = AngleAxisToRotationMatrix<T>(r1);

        T q_coeffs[4] = {q.w(), q.x(), q.y(), q.z()};
        Eigen::Matrix<T,3,3> R = QuaternionToRotation<T>(q_coeffs);

        /// 对应论文最后的整体refine
        // 对应论(2): 平移的残差 = (R(q_oi+1oi) - I) * p_oc - sj * R_oc * t_ci+1ci + t_oi+1oi
        Eigen::Matrix<T,3,1> t_err = (R1 - Eigen::Matrix<T,3,3>::Identity()) * t
                                     - (scale[0] * R * t2) + t1;

        // 对应论文(1): 旋转的残差 = q_oc^-1 * q_oi+1oi^-1 * q_oc * q_ci+1ci
        Eigen::Quaternion<T> q_err = q.conjugate() * q1 * q * q2.conjugate();

        T q_err_coeffs[4] = {q_err.w(), q_err.x(), q_err.y(), q_err.z()};
        Eigen::Matrix<T,3,3> R_err = QuaternionToRotation<T>(q_err_coeffs);

        T roll, pitch, yaw;
        mat2RPY(R_err, roll, pitch, yaw);

        residuals[0] = t_err(0);
        residuals[1] = t_err(1);
        residuals[2] = t_err(2);
        residuals[3] = roll;
        residuals[4] = pitch;
        residuals[5] = yaw;

        return true;
    }

private:
    Eigen::Vector3d m_rvec1, m_rvec2, m_tvec1, m_tvec2;
};

CamOdoCalibration::CamOdoCalibration()
 : mMinMotions(200)
 , mVerbose(false)
{

}

/**
 *
 * @param H_cam  存储(与图像帧匹配)相机每一帧的k-1帧到k帧得到变换矩阵
 * @param H_odo  存储(与图像帧匹配)里程计每一帧的k帧到k-1帧得到变换矩阵
 * @return
 */
bool CamOdoCalibration::addMotionSegment(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_cam,
                                    const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_odo)
{
    if (H_odo.size() != H_cam.size())
    {
        return false;
    }

    MotionSegment segment;

    // 对于每一帧
    for (size_t i = 0; i < H_odo.size(); ++i)
    {
        // 每一帧里程计的旋转
        Eigen::Matrix3d R_odo = H_odo.at(i).block<3,3>(0,0);
        Eigen::Vector3d t_odo = H_odo.at(i).block<3,1>(0,3);

        Motion odoMotion;
        // 里程计前后帧旋转的欧拉角
        odoMotion.rotation = RotationToAngleAxis(R_odo);
        // 里程计前后帧的平移
        odoMotion.translation = t_odo;

        segment.odoMotions.push_back(odoMotion);

        // 同理存储相机的运动(位姿)
        Eigen::Matrix3d R_cam = H_cam.at(i).block<3,3>(0,0);
        Eigen::Vector3d t_cam = H_cam.at(i).block<3,1>(0,3);

        Motion camMotion;
        camMotion.rotation = RotationToAngleAxis(R_cam);
        camMotion.translation = t_cam;

        // segment 中的相机运动
        segment.camMotions.push_back(camMotion);
    }

    // mSegment存储每一个segment
    mSegments.push_back(segment);

    return true;
}

// 统计每一个mSegments的odoMotions的个数
size_t CamOdoCalibration::getCurrentMotionCount(void) const
{
    size_t motionCount = 0;

    for (size_t i = 0; i < mSegments.size(); ++i)
    {
        motionCount += mSegments.at(i).odoMotions.size();
    }

    return motionCount;
}

size_t
CamOdoCalibration::getMotionCount(void) const
{
    return mMinMotions;
}

void
CamOdoCalibration::setMotionCount(size_t count)
{
    mMinMotions = count;
}

// 计算出外参
bool CamOdoCalibration::calibrate(Eigen::Matrix4d& H_cam_odo)
{
    std::vector<double> scales;

    std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > rvecsOdo, tvecsOdo, rvecsCam, tvecsCam;

    rvecsOdo.resize(mSegments.size());
    tvecsOdo.resize(mSegments.size());
    rvecsCam.resize(mSegments.size());
    tvecsCam.resize(mSegments.size());
    if (mSegments.empty())
    {
        std::cerr << "# ERROR: No segments, calibration fails!!" << std::endl;
        return false;
    }

    for (size_t i = 0; i < mSegments.size(); ++i)
    {
        MotionSegment& segment = mSegments.at(i);

        for (size_t j = 0; j < segment.odoMotions.size(); ++j)
        {
            // 记录每个segment中每个里程计的旋转
            rvecsOdo.at(i).push_back(segment.odoMotions.at(j).rotation);
            tvecsOdo.at(i).push_back(segment.odoMotions.at(j).translation);
            rvecsCam.at(i).push_back(segment.camMotions.at(j).rotation);
            tvecsCam.at(i).push_back(segment.camMotions.at(j).translation);
        }
    }

    // 计算出外参
    return estimate(rvecsOdo, tvecsOdo, rvecsCam, tvecsCam, H_cam_odo, scales);
}

bool
CamOdoCalibration::getVerbose(void)
{
    return mVerbose;
}

void
CamOdoCalibration::setVerbose(bool on)
{
    mVerbose = on;
}

/**
 *
 * @param rvecs1  记录每个segment中 每个里程计 的 旋转Vec
 * @param tvecs1  记录每个segment中 每个里程计 的 平移Vec
 * @param rvecs2  记录每个segment中 每个相机 的 旋转Vec
 * @param tvecs2  记录每个segment中 每个相机 的 平移Vec
 * @param H_cam_odo  输出值:相机到里程计的外参
 * @param scales     输出值:外参的尺度
 * @return
 */
bool CamOdoCalibration::estimate(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                            const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                            const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                            const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2,
                            Eigen::Matrix4d& H_cam_odo,
                            std::vector<double>& scales) const
{
    // Estimate R_yx first
    /// 先估计 R_yx; 对应论文求解roll和pitch

    Eigen::Matrix3d R_yx;
    estimateRyx(rvecs1, tvecs1, rvecs2, tvecs2, R_yx);

    // rvecs1.size()大小 = MotionSegment，MotionSegment为每次跟踪失败或停止前每一个相机帧对应的相机帧间位姿和里程计位姿
    int segmentCount = rvecs1.size();
    int motionCount = 0;
    for (int segmentId = 0; segmentId < segmentCount; ++segmentId)
    {
        motionCount += rvecs1.at(segmentId).size();
    }

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(motionCount * 2, 2 + segmentCount * 2);
    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(motionCount * 2, 1);

    int mark = 0;
    // 对每个segment求解论文的9,10
    for (int segmentId = 0; segmentId < segmentCount; ++segmentId)
    {
        // 每个segment
        for (size_t motionId = 0; motionId < rvecs1.at(segmentId).size(); ++motionId)
        {
            // 当前segment中 该帧 里程计的旋转Vec
            const Eigen::Vector3d& rvec1 = rvecs1.at(segmentId).at(motionId);

            const Eigen::Vector3d& tvec1 = tvecs1.at(segmentId).at(motionId);
            const Eigen::Vector3d& rvec2 = rvecs2.at(segmentId).at(motionId);
            const Eigen::Vector3d& tvec2 = tvecs2.at(segmentId).at(motionId);

            // Remove zero rotation.
            if (rvec1.norm() < 1e-10 || rvec2.norm() < 1e-10)
            {
                ++mark;
                continue;
            }

            Eigen::Quaterniond q1;

            // AngleAxisd:旋转向量（3X1）
            // 把旋转向量赋值给四元数q1
            // q1 = [ cosϕ  -sinϕ ]
            //      [ sinϕ   cosϕ ]
            q1 = Eigen::AngleAxisd(rvec1.norm(), rvec1.normalized());

            Eigen::Matrix2d J;
            // 对应论文（9）的J
            J = q1.toRotationMatrix().block<2,2>(0,0) - Eigen::Matrix2d::Identity();

            // project tvec2 to plane with normal defined by 3rd row of R_yx
            /// 将 相机的平移 投影到由R_yx 第三行（Z轴）定义为法线的平面
            Eigen::Vector3d n;
            n = R_yx.row(2);

            // pi是相机k到k+1帧的平移转到q_yx坐标的值 ??
            Eigen::Vector3d pi = R_yx * (tvec2 - tvec2.dot(n) * n);

            Eigen::Matrix2d K;
            // 对应论文（9）的K
            K << pi(0), -pi(1), pi(1), pi(0);
            std::cout << "mark : " << mark << std::endl;
            // 对应论文（10）
            G.block<2,2>(mark * 2, 0) = J;
            G.block<2,2>(mark * 2, 2 + segmentId * 2) = K;

            // 对应推导的（14）的w
            w.block<2,1>(mark * 2, 0) = tvec1.block<2,1>(0,0);

            ++mark;
        }
    }

    Eigen::MatrixXd m(2 + segmentCount * 2, 1);
    // 对应论文（9）
    m = G.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(w);

    // 对应论文（11）
    Eigen::Vector2d t(-m(0), -m(1));

    std::vector<double> alpha_hypos;
    // segmentId 对应 论文中的 VOsegment
    for (int segmentId = 0; segmentId < segmentCount; ++segmentId)
    {
        double alpha = atan2(m(2 + segmentId * 2 + 1), m(2 + segmentId * 2));
        double scale = m.block<2,1>(2 + segmentId * 2, 0).norm();

        alpha_hypos.push_back(alpha);
        scales.push_back(scale);
    }

    // 由于我们对于α(yaw)会有m个假设，可以通过最小化代价函数来计算最佳的α
    double errorMin = std::numeric_limits<double>::max();
    double alpha_best = 0.0;

    // 遍历所有的假设，取差值最小的α(yaw)
    for (size_t i = 0; i < alpha_hypos.size(); ++i)
    {
        double error = 0.0;
        double alpha = alpha_hypos.at(i);

        for (int segmentId = 0; segmentId < segmentCount; ++segmentId)
        {
            for (size_t motionId = 0; motionId < rvecs1.at(segmentId).size(); ++motionId)
            {
                const Eigen::Vector3d& rvec1 = rvecs1.at(segmentId).at(motionId);
                const Eigen::Vector3d& tvec1 = tvecs1.at(segmentId).at(motionId);
                const Eigen::Vector3d& rvec2 = rvecs2.at(segmentId).at(motionId);
                const Eigen::Vector3d& tvec2 = tvecs2.at(segmentId).at(motionId);

                // 对应论文(12)
                Eigen::Quaterniond q1;
                q1 = Eigen::AngleAxisd(rvec1.norm(), rvec1.normalized());

                Eigen::Matrix3d N;
                // 对应R(q_Oi+1Oi)-I
                N = q1.toRotationMatrix() - Eigen::Matrix3d::Identity();

                // 对应R(α)R(q_yx)
                Eigen::Matrix3d R = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ()) * R_yx;

                // project tvec2 to plane with normal defined by 3rd row of R
                /// 将 相机的帧间平移 投影到由R的第三行定义的法线的平面
                Eigen::Vector3d n;
                n = R.row(2);

                // 对应t_ck+1ck
                Eigen::Vector3d pc = tvec2 - tvec2.dot(n) * n;

                // 对应R(α)R(q_yx)t_ck+1ck
                Eigen::Vector3d A = R * pc;
                // 对应R(q_Oi+1Oi)-I×t_oc + t_Oi+1Oi
                Eigen::Vector3d b = N * (Eigen::Vector3d() << t, 0.0).finished() + tvec1;

                error += (A * scales.at(segmentId) - b).norm();
            }
        }

        if (error < errorMin)
        {
            errorMin = error;
            alpha_best = alpha;
        }
    }

    H_cam_odo.setIdentity();
    H_cam_odo.block<3,3>(0,0) = Eigen::AngleAxisd(alpha_best, Eigen::Vector3d::UnitZ()) * R_yx;
    H_cam_odo.block<2,1>(0,3) = t;

    if (mVerbose)
    {
        std::cout << "# INFO: Before refinement:" << std::endl;
        std::cout << "H_cam_odo = " << std::endl;
        std::cout << H_cam_odo << std::endl;
        std::cout << "scales = " << std::endl;
        for (size_t i = 0; i < scales.size(); ++i)
        {
            std::cout << scales.at(i) << " ";
        }
        std::cout << std::endl;
    }

    // 进行整体的refine
    refineEstimate(H_cam_odo, scales, rvecs1, tvecs1, rvecs2, tvecs2);

    if (mVerbose)
    {
        std::cout << "# INFO: After refinement:" << std::endl;
        std::cout << "H_cam_odo = " << std::endl;
        std::cout << H_cam_odo << std::endl;
        std::cout << "scales = " << std::endl;
        for (size_t i = 0; i < scales.size(); ++i)
        {
            std::cout << scales.at(i) << " ";
        }
        std::cout << std::endl;
    }

    return true;
}

/**
 * 估计R_yx
 * @param rvecs1  记录每个segment中每个里程计的旋转Vec
 * @param tvecs1  记录每个segment中每个里程计的平移Vec
 * @param rvecs2  记录每个segment中每个相机的旋转Vec
 * @param tvecs2  记录每个segment中每个相机的平移Vec
 * @param R_yx
 * @return
 */
bool CamOdoCalibration::estimateRyx(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                               const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                               const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                               const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2,
                               Eigen::Matrix3d& R_yx) const
{
    // 记录motion个数
    size_t motionCount = 0;
    for (size_t i = 0; i < rvecs1.size(); ++i)
    {
        motionCount += rvecs1.at(i).size();
    }

    Eigen::MatrixXd M(motionCount * 4, 4);
    M.setZero();

    size_t mark = 0;
    for (size_t i = 0; i < rvecs1.size(); ++i)
    {
        for (size_t j = 0; j < rvecs1.at(i).size(); ++j)
        {
            // 每个segment中每个里程计的旋转
            const Eigen::Vector3d& rvec1 = rvecs1.at(i).at(j);
            // 每个segment中每个相机的旋转
            const Eigen::Vector3d& rvec2 = rvecs2.at(i).at(j);


            // Remove zero rotation.
            if (rvec1.norm() == 0 || rvec2.norm() == 0)
            {
                continue;
            }

            Eigen::Quaterniond q1;
            q1 = Eigen::AngleAxisd(rvec1.norm(), rvec1.normalized());

            Eigen::Quaterniond q2;
            q2 = Eigen::AngleAxisd(rvec2.norm(), rvec2.normalized());

            // 论文(4)，论文(6)对应，四元数左右乘
            M.block<4,4>(mark * 4, 0) = QuaternionMultMatLeft(q1) - QuaternionMultMatRight(q2);

            ++mark;  // 对应计算的帧数
        }
    }

    // SVD分解求qyx
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // 对应论文（7），将q_yx用V的最后两列右奇异向量v3，v4表示：
    Eigen::Vector4d t1 = svd.matrixV().block<4,1>(0,2);
    Eigen::Vector4d t2 = svd.matrixV().block<4,1>(0,3);

    // solve constraint for q_yz: xy = -zw
    // 求解约束，对应论文（5）xy = -zw 虽然不太清楚为什么这么求解
    double s[2];
    if (!solveQuadraticEquation(t1(0) * t1(1) + t1(2) * t1(3),
                                t1(0) * t2(1) + t1(1) * t2(0) + t1(2) * t2(3) + t1(3) * t2(2),
                                t2(0) * t2(1) + t2(2) * t2(3),
                                s[0], s[1]))
    {
        std::cout << "# ERROR: Quadratic equation cannot be solved due to negative determinant." << std::endl;
        // 由于行列式为负，二次方程无法求解。
        return false;
    }

    Eigen::Matrix3d R_yxs[2];
    double yaw[2];

    for (int i = 0; i < 2; ++i)
    {
        double t = s[i] * s[i] * t1.dot(t1) + 2 * s[i] * t1.dot(t2) + t2.dot(t2);

        // solve constraint 求解约束
        double b = sqrt(1.0 / t); // 对应λ2
        double a = s[i] * b;         // 对应λ1

        Eigen::Quaterniond q_yx;
        // .coeffs : 以顺序为（x,y,z,w）表示
        q_yx.coeffs() = a * t1 + b * t2; // 对应q_yx = λ1*v3 + λ2*v4
        R_yxs[i] = q_yx.toRotationMatrix();

        double r, p;
        // 旋转矩阵转变为欧拉角
        mat2RPY(R_yxs[i], r, p, yaw[i]);
    }
    if (fabs(yaw[0]) < fabs(yaw[1]))
    {
        R_yx = R_yxs[0];
    }
    else
    {
        R_yx = R_yxs[1];
    }
    return true;
}

/**
 * 大优化Refine
 * @param H_cam_odo  相机到里程计的位姿
 * @param scales     尺度
 * @param rvecs1     记录每个segment中 每个里程计 的 旋转Vec
 * @param tvecs1     记录每个segment中 每个里程计 的 平移Vec
 * @param rvecs2     记录每个segment中 每个相机 的 旋转Vec
 * @param tvecs2     记录每个segment中 每个相机 的 平移Vec
 */
void CamOdoCalibration::refineEstimate(Eigen::Matrix4d& H_cam_odo, std::vector<double>& scales,
                                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2) const
{
    Eigen::Quaterniond q(H_cam_odo.block<3,3>(0,0));
    double q_coeffs[4] = {q.w(), q.x(), q.y(), q.z()};
    double t_coeffs[3] = {H_cam_odo(0,3), H_cam_odo(1,3), H_cam_odo(2,3)};

    ceres::Problem problem;

    for (size_t i = 0; i < rvecs1.size(); ++i)
    {
        for (size_t j = 0; j < rvecs1.at(i).size(); ++j)
        {
            ceres::CostFunction* costFunction =
                // t is only flexible on x and y.
                new ceres::AutoDiffCostFunction<CameraOdometerError, 6, 4, 2, 1>(
                        // 6：输出的残差的维度，4：旋转四元数，2：平移维度（固定了z轴？），1：尺度
                    new CameraOdometerError(rvecs1.at(i).at(j), tvecs1.at(i).at(j), rvecs2.at(i).at(j), tvecs2.at(i).at(j)));

            // 优化外参的四元数，外参的平移，外参的尺度
            problem.AddResidualBlock(costFunction, NULL, q_coeffs, t_coeffs, &scales.at(i));
        }
    }

    // 允许用户定义参数块并将其与它们所属的流形相关联
    ceres::LocalParameterization* quaternionParameterization =
        new ceres::QuaternionParameterization;

    problem.SetParameterization(q_coeffs, quaternionParameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
//    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (mVerbose)
    {
        std::cout << "camodocalibration: Ceres: " << std::endl;
        std::cout << summary.BriefReport() << std::endl;
    }

    // 优化了外参的平移和旋转
    q = Eigen::Quaterniond(q_coeffs[0], q_coeffs[1], q_coeffs[2], q_coeffs[3]);
    H_cam_odo.block<3,1>(0,3) << t_coeffs[0], t_coeffs[1], t_coeffs[2];
    H_cam_odo.block<3,3>(0,0) = q.toRotationMatrix();
}

// 解二次方程： ax^2+bx+c =0 =>x1,2 = [-b+-sqrt(b^2-4ac)] / 2a
bool CamOdoCalibration::solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const
{
    if (fabs(a) < 1e-12)
    {
        x1 = x2 = -c / b;
        return true;
    }
    double delta2 = b * b - 4.0 * a * c;

    if (delta2 < 0.0)
    {
        return false;
    }

    double delta = sqrt(delta2);

    x1 = (-b + delta) / (2.0 * a);
    x2 = (-b - delta) / (2.0 * a);

    return true;
}

}
