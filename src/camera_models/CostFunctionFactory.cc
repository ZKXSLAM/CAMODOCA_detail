#include "CostFunctionFactory.h"

#include "ceres/ceres.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

namespace camodocal
{
/**
 * 把里程计的位姿从世界坐标系转到相机坐标系
 * @tparam T
 * @param q_cam_odo     qoc:  相机到里程计坐标系的旋转四元数
 * @param t_cam_odo     toc:  相机到里程计坐标系的平移
 * @param p_odo      t_{Wo}o: 里程计相对于里程计第一帧的平移
 * @param att_odo  Att_{Wo}o: 里程计相对于里程计第一帧的欧拉角
 * @param q
 * @param t
 * @param optimize_cam_odo_z   是否优化外参z轴
 */
template<typename T>
void worldToCameraTransform(const T* const q_cam_odo, const T* const t_cam_odo,
                       const T* const p_odo, const T* const att_odo,
                       T* q, T* t, bool optimize_cam_odo_z = true)
{
    /// 通过取里程计相对于里程计第一帧的四元数的逆获得从里程计第一帧(里程计世界坐标)到里程计当前帧的旋转
    // q_inv : q_o{Wo}
    // q_z(γ)
///为什么sin的地方是负号 : 因为这里旋转的逆，q^-1 = q*/||q||^2 (||q|| = 1) => q^-1 = q*; q*即为四元数的共轭，即把虚部变成相反数
    Eigen::Quaternion<T> q_z_inv(cos(att_odo[0] / T(2)), T(0), T(0), -sin(att_odo[0] / T(2)));
    // q_y(β)
    Eigen::Quaternion<T> q_y_inv(cos(att_odo[1] / T(2)), T(0), -sin(att_odo[1] / T(2)), T(0));
    // q_x(α)
    Eigen::Quaternion<T> q_x_inv(cos(att_odo[2] / T(2)), -sin(att_odo[2] / T(2)), T(0), T(0));

    // 从里程计第一帧(里程计世界坐标)到里程计当前帧的旋转 表示为绕ZYX定轴分解的连乘形式
    // q_o{Wo}
    Eigen::Quaternion<T> q_zyx_inv = q_x_inv * q_y_inv * q_z_inv;

    // q_o{Wo}
    T q_odo[4] = {q_zyx_inv.w(), q_zyx_inv.x(), q_zyx_inv.y(), q_zyx_inv.z()};

    // qco = qoc^-1
    T q_odo_cam[4] = {q_cam_odo[3], -q_cam_odo[0], -q_cam_odo[1], -q_cam_odo[2]};

    T q0[4];
    // 通过qco 把里程计相对于里程计第一帧(里程计世界坐标)的旋转 转变为 里程计第一帧(里程计世界坐标)到相机坐标系的旋转
    // 四元数相乘 q0 = qc{Wo} = qco × q_o{Wo}
    ceres::QuaternionProduct(q_odo_cam, q_odo, q0);

    T t0[3];
    // t_{Wo}o 里程计相对于里程计第一帧的平移
    T t_odo[3] = {p_odo[0], p_odo[1], p_odo[2]};

    // 通过 把里程计相对于里程计第一帧(里程计世界坐标)的旋转 把 里程计当前帧对里程计第一帧(里程计世界坐标)的平移 转化为
    // 通过四元数q旋转点,t0 = too = q_o{Wo} * t_{Wo}o
    ceres::QuaternionRotatePoint(q_odo, t_odo, t0);

    // tow += too + toc
    t0[0] += t_cam_odo[0];
    t0[1] += t_cam_odo[1];

    // 如果优化cam-odo的z轴，就加上tco的z轴
    if (optimize_cam_odo_z)
    {
        t0[2] += t_cam_odo[2];
    }

    // t = tcw = qco * tow
    ceres::QuaternionRotatePoint(q_odo_cam, t0, t);
    //
    t[0] = -t[0]; t[1] = -t[1]; t[2] = -t[2];

    // Convert quaternion from Ceres convention (w, x, y, z) to Eigen convention (x, y, z, w)
    q[0] = q0[1]; q[1] = q0[2]; q[2] = q0[3]; q[3] = q0[0];
}

template<class CameraT>
class ReprojectionError1
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ReprojectionError1(const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p)
     : m_observed_P(observed_P), m_observed_p(observed_p)
     , m_sqrtPrecisionMat(Eigen::Matrix2d::Identity()) {}

    ReprojectionError1(const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p,
                       const Eigen::Matrix2d& sqrtPrecisionMat)
     : m_observed_P(observed_P), m_observed_p(observed_p)
     , m_sqrtPrecisionMat(sqrtPrecisionMat) {}

    ReprojectionError1(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p)
     : m_intrinsic_params(intrinsic_params)
     , m_observed_P(observed_P), m_observed_p(observed_p) {}

    // variables: camera intrinsics and camera extrinsics
    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q,
                    const T* const t,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q, t, P, predicted_p);

        Eigen::Matrix<T,2,1> e = predicted_p - m_observed_p.cast<T>();

        Eigen::Matrix<T,2,1> e_weighted = m_sqrtPrecisionMat.cast<T>() * e;

        residuals[0] = e_weighted(0);
        residuals[1] = e_weighted(1);

        return true;
    }

    // variables: camera-odometry transforms and odometry poses
    template <typename T>
    bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const p_odo, const T* const att_odo,
                    T* residuals) const
    {
        T q[4], t[3];
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, q, t);

        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // camera intrinsics
    std::vector<double> m_intrinsic_params;

    // observed 3D point
    Eigen::Vector3d m_observed_P;

    // observed 2D point
    Eigen::Vector2d m_observed_p;

    // square root of precision matrix
    Eigen::Matrix2d m_sqrtPrecisionMat;
};

// variables: camera extrinsics, 3D point
template<class CameraT>
class ReprojectionError2
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 传入相机参数 和 观察到的像素点坐标
    ReprojectionError2(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector2d& observed_p)
     : m_intrinsic_params(intrinsic_params), m_observed_p(observed_p) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t,
                    const T* const point, T* residuals) const
    {
        Eigen::Matrix<T,3,1> P;
        P(0) = T(point[0]);
        P(1) = T(point[1]);
        P(2) = T(point[2]);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());

        // project 3D object point to the image plane 将三维对象点投影到图像平面
        Eigen::Matrix<T,2,1> predicted_p;
        // 把三维点投影到像素坐标系
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        // 残差是预测位置和观测位置之间的差值
        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // camera intrinsics 相机内参
    std::vector<double> m_intrinsic_params;

    // observed 2D point 观测到的像素点坐标
    Eigen::Vector2d m_observed_p;
};

template<class CameraT>
class ReprojectionError3
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ReprojectionError3(const Eigen::Vector2d& observed_p)
     : m_observed_p(observed_p)
     , m_sqrtPrecisionMat(Eigen::Matrix2d::Identity())
     , m_optimize_cam_odo_z(true) {}

    ReprojectionError3(const Eigen::Vector2d& observed_p,
                       const Eigen::Matrix2d& sqrtPrecisionMat)
     : m_observed_p(observed_p)
     , m_sqrtPrecisionMat(sqrtPrecisionMat)
     , m_optimize_cam_odo_z(true){}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector2d& observed_p)
     : m_intrinsic_params(intrinsic_params)
     , m_observed_p(observed_p)
     , m_sqrtPrecisionMat(Eigen::Matrix2d::Identity())
     , m_optimize_cam_odo_z(true) {}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector2d& observed_p,
                       const Eigen::Matrix2d& sqrtPrecisionMat)
     : m_intrinsic_params(intrinsic_params)
     , m_observed_p(observed_p)
     , m_sqrtPrecisionMat(sqrtPrecisionMat)
     , m_optimize_cam_odo_z(true) {}


    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector3d& odo_pos,
                       const Eigen::Vector3d& odo_att,
                       const Eigen::Vector2d& observed_p,
                       bool optimize_cam_odo_z)
     : m_intrinsic_params(intrinsic_params)
     , m_odo_pos(odo_pos), m_odo_att(odo_att)
     , m_observed_p(observed_p)
     , m_optimize_cam_odo_z(optimize_cam_odo_z) {}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Quaterniond& cam_odo_q,
                       const Eigen::Vector3d& cam_odo_t,
                       const Eigen::Vector3d& odo_pos,
                       const Eigen::Vector3d& odo_att,
                       const Eigen::Vector2d& observed_p)
     : m_intrinsic_params(intrinsic_params)
     , m_cam_odo_q(cam_odo_q), m_cam_odo_t(cam_odo_t)
     , m_odo_pos(odo_pos), m_odo_att(odo_att)
     , m_observed_p(observed_p)
     , m_optimize_cam_odo_z(true) {}

    // variables: camera intrinsics, camera-to-odometry transform, odometry extrinsics, 3D point
    // 优化相机内参，相机-里程计外参，里程计外参，三维点
    /**
     * 计算残差，
     * @tparam T
     * @param intrinsic_params   相机内参
     * @param q_cam_odo    q_oc: 相机坐标系到里程计坐标系的旋转四元数(4维)
     * @param t_cam_odo    t_oc: 相机坐标系到里程计坐标系的平移(3维)
     * @param p_odo      t_{wo}o:里程计相对于里程计第一帧的平移(没固定前：3维)
     * @param att_odo  Att_{wo}o:里程计相对于里程计第一帧的的欧拉角(没固定前：3维)
     * @param point              三维点(3维)
     * @param residuals          残差(2维)
     * @return
     */
    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const p_odo, const T* const att_odo,
                    const T* const point, T* residuals) const
    {
        T q[4], t[3];
        // 通过Toc和 T{wo}o 得到q_c{wo},t_c{wo}
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, q, t, m_optimize_cam_odo_z);

        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane 将三维点投影到图像平面
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q, t, P, predicted_p);

        Eigen::Matrix<T,2,1> err = predicted_p - m_observed_p.cast<T>();
        // 给误差添加权重
        Eigen::Matrix<T,2,1> err_weighted = m_sqrtPrecisionMat.cast<T>() * err;

        residuals[0] = err_weighted(0);
        residuals[1] = err_weighted(1);

        return true;
    }

    // variables: camera-to-odometry transform, 3D point
    /**
     *
     * @tparam T
     * @param q_cam_odo   qoc
     * @param t_cam_odo   toc
     * @param point
     * @param residuals
     * @return
     */
    template <typename T>
    bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const point, T* residuals) const
    {
        // 思考 m_odo_pos 与上面的 p_odo 的区别 ：其实差不多， m_odo_pos 也是由 odo_pos赋值的
        T p_odo[3] = {T(m_odo_pos(0)), T(m_odo_pos(1)), T(m_odo_pos(2))};
        T att_odo[3] = {T(m_odo_att(0)), T(m_odo_att(1)), T(m_odo_att(2))};
        T q[4], t[3];

        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, q, t, m_optimize_cam_odo_z);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);
        /// 注意 没有乘权重
        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

    // variables: camera-to-odometry transform, odometry extrinsics, 3D point
    /**
     * 计算残差，优化相机-里程计外参，里程计外参，三维点
     * @tparam T
     * @param q_cam_odo
     * @param t_cam_odo
     * @param p_odo
     * @param att_odo
     * @param point
     * @param residuals
     * @return
     */
    template <typename T> bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const p_odo, const T* const att_odo,
                    const T* const point, T* residuals) const
    {
        T q[4], t[3];
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, q, t, m_optimize_cam_odo_z);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        Eigen::Matrix<T,2,1> err = predicted_p - m_observed_p.cast<T>();
        Eigen::Matrix<T,2,1> err_weighted = m_sqrtPrecisionMat.cast<T>() * err;

        residuals[0] = err_weighted(0);
        residuals[1] = err_weighted(1);

        return true;
    }

    // variables: 3D point
    template <typename T>
    bool operator()(const T* const point, T* residuals) const
    {
        T q_cam_odo[4] = {T(m_cam_odo_q.coeffs()(0)), T(m_cam_odo_q.coeffs()(1)), T(m_cam_odo_q.coeffs()(2)), T(m_cam_odo_q.coeffs()(3))};
        T t_cam_odo[3] = {T(m_cam_odo_t(0)), T(m_cam_odo_t(1)), T(m_cam_odo_t(2))};
        T p_odo[3] = {T(m_odo_pos(0)), T(m_odo_pos(1)), T(m_odo_pos(2))};
        T att_odo[3] = {T(m_odo_att(0)), T(m_odo_att(1)), T(m_odo_att(2))};
        T q[4], t[3];

        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, q, t, m_optimize_cam_odo_z);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // camera intrinsics
    std::vector<double> m_intrinsic_params;

    // observed camera-odometry transform
    // 观测的相机-里程计位姿变换
    Eigen::Quaterniond m_cam_odo_q;
    Eigen::Vector3d m_cam_odo_t;

    // observed odometry
    Eigen::Vector3d m_odo_pos;
    Eigen::Vector3d m_odo_att;

    // observed 2D point
    Eigen::Vector2d m_observed_p;

    // 权重
    Eigen::Matrix2d m_sqrtPrecisionMat;

    // 是否优化外参的z轴
    bool m_optimize_cam_odo_z;
};

// variables: camera intrinsics and camera extrinsics
template<class CameraT>
class StereoReprojectionError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoReprojectionError(const Eigen::Vector3d& observed_P,
                            const Eigen::Vector2d& observed_p_l,
                            const Eigen::Vector2d& observed_p_r)
     : m_observed_P(observed_P)
     , m_observed_p_l(observed_p_l)
     , m_observed_p_r(observed_p_r)
    {

    }

    template <typename T>
    bool operator()(const T* const intrinsic_params_l,
                    const T* const intrinsic_params_r,
                    const T* const q_l,
                    const T* const t_l,
                    const T* const q_l_r,
                    const T* const t_l_r,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P;
        P(0) = T(m_observed_P(0));
        P(1) = T(m_observed_P(1));
        P(2) = T(m_observed_P(2));

        Eigen::Matrix<T,2,1> predicted_p_l;
        CameraT::spaceToPlane(intrinsic_params_l, q_l, t_l, P, predicted_p_l);

        Eigen::Quaternion<T> q_r = Eigen::Quaternion<T>(q_l_r) * Eigen::Quaternion<T>(q_l);

        Eigen::Matrix<T,3,1> t_r;
        t_r(0) = t_l[0];
        t_r(1) = t_l[1];
        t_r(2) = t_l[2];

        t_r = Eigen::Quaternion<T>(q_l_r) * t_r;
        t_r(0) += t_l_r[0];
        t_r(1) += t_l_r[1];
        t_r(2) += t_l_r[2];

        Eigen::Matrix<T,2,1> predicted_p_r;
        CameraT::spaceToPlane(intrinsic_params_r, q_r.coeffs().data(), t_r.data(), P, predicted_p_r);

        residuals[0] = predicted_p_l(0) - T(m_observed_p_l(0));
        residuals[1] = predicted_p_l(1) - T(m_observed_p_l(1));
        residuals[2] = predicted_p_r(0) - T(m_observed_p_r(0));
        residuals[3] = predicted_p_r(1) - T(m_observed_p_r(1));

        return true;
    }

private:
    // observed 3D point
    Eigen::Vector3d m_observed_P;

    // observed 2D point
    Eigen::Vector2d m_observed_p_l;
    Eigen::Vector2d m_observed_p_r;
};

boost::shared_ptr<CostFunctionFactory> CostFunctionFactory::m_instance;

CostFunctionFactory::CostFunctionFactory()
{

}

boost::shared_ptr<CostFunctionFactory>
CostFunctionFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CostFunctionFactory);
    }

    return m_instance;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector3d& observed_P,
                                          const Eigen::Vector2d& observed_p,
                                          int flags) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    switch (flags)
    {
    case CAMERA_INTRINSICS | CAMERA_POSE:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<EquidistantCamera>, 2, 8, 4, 3>(
                    new ReprojectionError1<EquidistantCamera>(observed_P, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<PinholeCamera>, 2, 8, 4, 3>(
                    new ReprojectionError1<PinholeCamera>(observed_P, observed_p));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<CataCamera>, 2, 9, 4, 3>(
                    new ReprojectionError1<CataCamera>(observed_P, observed_p));
            break;
        case Camera::SCARAMUZZA:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 3>(
                    new ReprojectionError1<OCAMCamera>(observed_P, observed_p));
            break;
        }
        break;
    case CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<EquidistantCamera>, 2, 4, 3, 3, 3>(
                    new ReprojectionError1<EquidistantCamera>(intrinsic_params, observed_P, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<PinholeCamera>, 2, 4, 3, 3, 3>(
                    new ReprojectionError1<PinholeCamera>(intrinsic_params, observed_P, observed_p));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<CataCamera>, 2, 4, 3, 3, 3>(
                    new ReprojectionError1<CataCamera>(intrinsic_params, observed_P, observed_p));
            break;
        case Camera::SCARAMUZZA:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<OCAMCamera>, 2, 4, 3, 3, 3>(
                    new ReprojectionError1<OCAMCamera>(intrinsic_params, observed_P, observed_p));
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector3d& observed_P,
                                          const Eigen::Vector2d& observed_p,
                                          const Eigen::Matrix2d& sqrtPrecisionMat,
                                          int flags) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    switch (flags)
    {
    case CAMERA_INTRINSICS | CAMERA_POSE:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<EquidistantCamera>, 2, 8, 4, 3>(
                    new ReprojectionError1<EquidistantCamera>(observed_P, observed_p, sqrtPrecisionMat));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<PinholeCamera>, 2, 8, 4, 3>(
                    new ReprojectionError1<PinholeCamera>(observed_P, observed_p, sqrtPrecisionMat));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<CataCamera>, 2, 9, 4, 3>(
                    new ReprojectionError1<CataCamera>(observed_P, observed_p, sqrtPrecisionMat));
            break;
        case Camera::SCARAMUZZA:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 3>(
                    new ReprojectionError1<OCAMCamera>(observed_P, observed_p, sqrtPrecisionMat));
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction* CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector2d& observed_p,
                                          int flags, bool optimize_cam_odo_z) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params); // 读取相机内参

    switch (flags)
    {
    //优化相机位姿和三维点
    case CAMERA_POSE | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError2<EquidistantCamera>, 2, 4, 3, 3>(
                    new ReprojectionError2<EquidistantCamera>(intrinsic_params, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError2<PinholeCamera>, 2, 4, 3, 3>(
                    new ReprojectionError2<PinholeCamera>(intrinsic_params, observed_p));
            break;
            /// <2,4,3,3>: 2 :输出维度,2维的残差; 4: 输入维度，相机的旋转四元数; 3: 输入维度，相机的平移; 3: 输入维度，三维点的坐标
            /// 对应于 bool operator()(const T* const q, const T* const t,  const T* const point, T* residuals) const
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError2<CataCamera>, 2, 4, 3, 3>(
                    new ReprojectionError2<CataCamera>(intrinsic_params, observed_p));
            break;
        case Camera::SCARAMUZZA:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError2<OCAMCamera>, 2, 4, 3, 3>(
                    new ReprojectionError2<OCAMCamera>(intrinsic_params, observed_p));
            break;
        }
        break;
    // 优化外参，里程计3位姿和三维点
    case CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_3D_POSE | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p));
            }
            break;
        case Camera::PINHOLE:
            // 是否优化外参z轴平移
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p));
                /// <2, 4, 3, 2, 1, 3>: 2 :输出维度,2维的残差; 4: 输入维度，相机的旋转四元数;3: 输入维度，相机的平移;
                ///  2: 输入维度，里程计的平移(原本是3维固定住z轴所以是2维);
                ///  1:输入维度，里程计的欧拉角(原本是3维固定住roll，pitch只优化yaw所以是1维; 3:输入维度，三维点的坐标
                /// 对应于：bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                ///                    const T* const p_odo, const T* const att_odo,
                ///                    const T* const point, T* residuals) const
            }
            else
            {
                costFunction =                                      /// 因为不优化外参的z，所以外参的输入维度是2
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p));
            }
            break;
        case Camera::SCARAMUZZA:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, observed_p));
            }
            break;
        }
        break;
    // 优化外参，里程计6位姿和三维点
    case CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p));
                /// <2, 4, 3, 3, 3, 3>: 2 :输出维度,2维的残差; 4: 输入维度，相机的旋转四元数;3: 输入维度，相机的平移;
                ///  3: 输入维度，里程计的平移(没固定住z轴); 3:输入维度，里程计的欧拉角(没固定住roll，pitch);
                ///  3:输入维度，三维点的坐标
                /// 对应于：bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                ///                    const T* const p_odo, const T* const att_odo,
                ///                    const T* const point, T* residuals) const
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p));
            }
            break;
        case Camera::SCARAMUZZA:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, observed_p));
            }
            break;
        }
        break;
    // 优化内参，外参，里程计3位姿和三维点
    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_3D_POSE | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 8, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<EquidistantCamera>(observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p));
                /// <2, 8, 4, 3, 2, 1, 3>: 2 :输出维度,2维的残差; 8: 输入维度，相机内参
                ///  4: 输入维度，相机的旋转四元数;3: 输入维度，相机的平移;
                ///  2: 输入维度，里程计的平移(原本是3维固定住z轴所以是2维);
                ///  1:输入维度，里程计的欧拉角(原本是3维固定住roll，pitch只优化yaw所以是1维; 3:输入维度，三维点的坐标
                ///
                /// bool operator()(const T* const intrinsic_params,
                //                    const T* const q_cam_odo, const T* const t_cam_odo,
                //                    const T* const p_odo, const T* const att_odo,
                //                    const T* const point, T* residuals) const
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(observed_p));
            }
            break;
        case Camera::SCARAMUZZA:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<OCAMCamera>(observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<OCAMCamera>(observed_p));
            }
            break;
        }
        break;
    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 8, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(observed_p));
            }
            break;
        case Camera::SCARAMUZZA:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(observed_p));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(observed_p));
            }
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector2d& observed_p,
                                          const Eigen::Matrix2d& sqrtPrecisionMat,
                                          int flags, bool optimize_cam_odo_z) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    switch (flags)
    {
    case CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            break;
        case Camera::SCARAMUZZA:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, observed_p, sqrtPrecisionMat));
            }
            break;
        }
        break;
    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 8, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, sqrtPrecisionMat));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, sqrtPrecisionMat));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, sqrtPrecisionMat));
            }
            break;
        case Camera::SCARAMUZZA:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(observed_p, sqrtPrecisionMat));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(observed_p, sqrtPrecisionMat));
            }
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector3d& odo_pos,
                                          const Eigen::Vector3d& odo_att,
                                          const Eigen::Vector2d& observed_p,
                                          int flags, bool optimize_cam_odo_z) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    switch (flags)
    {
    case CAMERA_ODOMETRY_TRANSFORM | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::SCARAMUZZA:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<OCAMCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_cam_odo_z));
            }
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Quaterniond& cam_odo_q,
                                          const Eigen::Vector3d& cam_odo_t,
                                          const Eigen::Vector3d& odo_pos,
                                          const Eigen::Vector3d& odo_att,
                                          const Eigen::Vector2d& observed_p,
                                          int flags) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    switch (flags)
    {
    case POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 3>(
                    new ReprojectionError3<EquidistantCamera>(intrinsic_params, cam_odo_q, cam_odo_t, odo_pos, odo_att, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 3>(
                    new ReprojectionError3<PinholeCamera>(intrinsic_params, cam_odo_q, cam_odo_t, odo_pos, odo_att, observed_p));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 3>(
                    new ReprojectionError3<CataCamera>(intrinsic_params, cam_odo_q, cam_odo_t, odo_pos, odo_att, observed_p));
            break;
        case Camera::SCARAMUZZA:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError3<OCAMCamera>, 2, 3>(
                    new ReprojectionError3<OCAMCamera>(intrinsic_params, cam_odo_q, cam_odo_t, odo_pos, odo_att, observed_p));
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& cameraL,
                                          const CameraConstPtr& cameraR,
                                          const Eigen::Vector3d& observed_P,
                                          const Eigen::Vector2d& observed_p_l,
                                          const Eigen::Vector2d& observed_p_r) const
{
    ceres::CostFunction* costFunction = 0;

    if (cameraL->modelType() != cameraR->modelType())
    {
        return costFunction;
    }

    switch (cameraL->modelType())
    {
    case Camera::KANNALA_BRANDT:
        costFunction =
            new ceres::AutoDiffCostFunction<StereoReprojectionError<EquidistantCamera>, 4, 8, 8, 4, 3, 4, 3>(
                new StereoReprojectionError<EquidistantCamera>(observed_P, observed_p_l, observed_p_r));
        break;
    case Camera::PINHOLE:
        costFunction =
            new ceres::AutoDiffCostFunction<StereoReprojectionError<PinholeCamera>, 4, 8, 8, 4, 3, 4, 3>(
                new StereoReprojectionError<PinholeCamera>(observed_P, observed_p_l, observed_p_r));
        break;
    case Camera::MEI:
        costFunction =
            new ceres::AutoDiffCostFunction<StereoReprojectionError<CataCamera>, 4, 9, 9, 4, 3, 4, 3>(
                new StereoReprojectionError<CataCamera>(observed_P, observed_p_l, observed_p_r));
        break;
    case Camera::SCARAMUZZA:
        costFunction =
            new ceres::AutoDiffCostFunction<StereoReprojectionError<OCAMCamera>, 4, SCARAMUZZA_CAMERA_NUM_PARAMS, SCARAMUZZA_CAMERA_NUM_PARAMS, 4, 3, 4, 3>(
                new StereoReprojectionError<OCAMCamera>(observed_P, observed_p_l, observed_p_r));
        break;
    }

    return costFunction;
}

}

