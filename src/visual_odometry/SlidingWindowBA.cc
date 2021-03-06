#include "SlidingWindowBA.h"

#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <cstdio>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <camodocal/sparse_graph/SparseGraphUtils.h>
#include "ceres/ceres.h"
#include "../camera_models/CostFunctionFactory.h"
#include "camodocal/EigenUtils.h"
#include "../npoint/five-point/five-point.hpp"
#include "../pose_estimation/P3P.h"

namespace camodocal
{

SlidingWindowBA::SlidingWindowBA(const CameraConstPtr& camera,
                                 int N, int n, int mode,
                                 Eigen::Matrix4d globalCameraPose)
 : m_N(N)
 , m_n(n)
 , m_mode(mode)
 , k_camera(camera)
 , k_epipolarThresh(0.00005)
 , k_minDisparity(3.0)
 , k_nominalFocalLength(300.0)
 , k_reprojErrorThresh(2.0)
 , m_frameCount(0)
 , m_verbose(false)
 , k_min2D2DFeatureCorrespondences(10)
 , k_min2D3DFeatureCorrespondences(10)
{
    const Eigen::Matrix4d& H_cam_odo = globalCameraPose;

    m_T_cam_odo.rotation() = Eigen::Quaterniond(H_cam_odo.block<3,3>(0,0));
    m_T_cam_odo.translation() = Eigen::Vector3d(H_cam_odo.block<3,1>(0,3));
}

Eigen::Matrix4d
SlidingWindowBA::globalCameraPose(void)
{
    return m_T_cam_odo.toMatrix();
}

bool SlidingWindowBA::addFrame(FramePtr& frame)
{
    FramePtr frameCurr = frame;

    if (m_mode == VO)
    {
        frameCurr->cameraPose() = boost::make_shared<Pose>();
    }

    // 将当前帧添加给滑窗
    m_window.push_back(frameCurr);

    // m_N : 20
    // 如果滑窗内的帧数大于阈值，就将之前的帧丢弃
    while ((int)m_window.size() > m_N)
    {
        m_window.pop_front();
    }

    // 统计帧的总数
    ++m_frameCount;

    if (m_verbose)
    {
        // 一共添加了30次，初始四个相机各输出了一次
        // Sling windows已添加 .. 帧
        std::cout << "# INFO: Sliding windows : Added frame " << m_frameCount - 1 << "." << std::endl;
    }

    if (m_frameCount == 1) // 如果当前添加帧为第一帧
    {
        if (m_mode == VO)
        {
            // 设定第一帧的相机坐标系位姿为世界坐标系原点
            frameCurr->cameraPose()->rotation() = Eigen::Quaterniond::Identity();
            frameCurr->cameraPose()->translation().setZero();
        }

        return true;
    }

    // 上一帧就是窗口中所有帧的倒数第二帧
    FramePtr framePrev = *(++m_window.rbegin());

    // find feature correspondences between previous and current frames
    /// 查找上一帧和当前帧之间的特征对应

    std::vector<std::vector<Point2DFeaturePtr> > featureCorrespondences; //2d匹配特征点的集合
    // 把满足要求的匹配点 存放到 2d匹配特征点的集合
    findFeatureCorrespondences(frameCurr->features2D(), 2, featureCorrespondences);
    // 把当前帧和上一帧的所有的最佳匹配ID设为-1
    for (size_t i = 0; i < framePrev->features2D().size(); ++i)
    {
        framePrev->features2D().at(i)->bestNextMatchId() = -1;
    }
    for (size_t i = 0; i < frameCurr->features2D().size(); ++i)
    {
        frameCurr->features2D().at(i)->bestPrevMatchId() = -1;
    }

    if (m_verbose)
    {
        // 在最新2帧中找到 ...  个特征对应
        std::cout << "# INFO: Sliding windows : Found " << featureCorrespondences.size() << " feature correspondences in last 2 frames." << std::endl;
    }

    // 如果是初始的前两帧
    if (m_frameCount == 2)
    {
        // compute pose in frame 1 relative to frame 0
        /// 计算帧1中相对于帧0的位姿

        // 存放一帧图像的像素点
        std::vector<cv::Point2f> imagePoints[2];


        for (size_t i = 0; i < featureCorrespondences.size(); ++i)
        {
            //对于每一对匹配点对，分别放在imagePoints[0],imagePoints[1]
            std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

            //fc.size() : 2
            for (size_t j = 0; j < fc.size(); ++j)
            {
                imagePoints[j].push_back(fc.at(j)->keypoint().pt);
            }
        }

        // 如果2D-2D匹配数量不足
        if ((int)imagePoints[0].size() < k_min2D2DFeatureCorrespondences)
        {
            if (m_verbose)
            {
                // 用于BA初始化的2D-2D匹配数量不足
                std::cout << "# INFO: sliding windows :Insufficient number of 2D-2D correspondences for BA initialization." << std::endl;
            }

            return false;
        }

        std::vector<cv::Point2f> rectImagePoints[2];
        for (size_t i = 0; i < 2; ++i)
        {
            // 图像点矫正
            rectifyImagePoints(k_camera, imagePoints[i], rectImagePoints[i]);
        }

        cv::Mat inliers;
        if (m_mode == VO)
        {
            cv::Mat E, R_cv, t_cv;
            // 找到两帧之间本质矩阵
            E = findEssentialMat(rectImagePoints[0], rectImagePoints[1], 1.0, cv::Point2d(0.0, 0.0),
                                 CV_FM_RANSAC, 0.99, k_reprojErrorThresh / k_nominalFocalLength, 100, inliers);
            // 从本质矩阵中恢复出位姿
            recoverPose(E, rectImagePoints[0], rectImagePoints[1], R_cv, t_cv, 1.0, cv::Point2d(0.0, 0.0), inliers);

            if (m_verbose)
            {
                std::cout << "# INFO: sliding windows : Computed pose in frame 0 wrt pose in frame 1 with " << cv::countNonZero(inliers) << " inliers:" << std::endl;
                std::cout << "R_cv : " << std::endl << R_cv << std::endl;
                std::cout <<  "t_cv : " << std::endl << t_cv << std::endl;
            }

            Eigen::Matrix3d R;
            cv::cv2eigen(R_cv, R);

            Eigen::Vector3d t;
            cv::cv2eigen(t_cv, t);

            // 当前帧相机的位姿赋值 (R_kk+1)
            frameCurr->cameraPose()->rotation() = Eigen::Quaterniond(R);
            frameCurr->cameraPose()->translation() = t;
        }
        else
        {
            inliers = cv::Mat(1, featureCorrespondences.size(), CV_8U);
            inliers = cv::Scalar(1);
        }

        // 所有点对的集合
        std::vector<std::vector<Point2DFeaturePtr> > inlierFeatureCorrespondences;
        for (int i = 0; i < inliers.cols; ++i)
        {
            if (!inliers.at<unsigned char>(0,i))
            {
                continue;
            }

            inlierFeatureCorrespondences.push_back(featureCorrespondences.at(i));
        }

        //  图像点清空
        for (int i = 0; i < 2; ++i)
        {
            imagePoints[i].clear();
        }

        // 对于每一个匹配点对
        for (size_t i = 0; i < inlierFeatureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = inlierFeatureCorrespondences.at(i);

            // inlierFeatureCorrespondences: fc.size() : 2
            for (size_t j = 0; j < fc.size(); ++j)
            {
                imagePoints[j].push_back(fc.at(j)->keypoint().pt);
            }
        }

        for (size_t i = 0; i < 2; ++i)
        {
            rectifyImagePoints(k_camera, imagePoints[i], rectImagePoints[i]);
        }

        // triangulate scene points
        /// 三角化场景点
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points3D;
        std::vector<size_t> indices;

        // m_mode : 1 (VO = 1, ODOMETRY = 0)
        if (m_mode == VO)
        {
            triangulatePoints(framePrev->cameraPose()->rotation(), framePrev->cameraPose()->translation(), imagePoints[0],
                              frameCurr->cameraPose()->rotation(), frameCurr->cameraPose()->translation(), imagePoints[1],
                              points3D, indices);
        }
        else // m_mode == ODOMETRY
        {
            Eigen::Matrix4d H_odo_cam = m_T_cam_odo.toMatrix().inverse();

            // 上一帧相机在世界坐标系下的位姿 = Tco × Tow
            Eigen::Matrix4d H1 = H_odo_cam * framePrev->systemPose()->toMatrix().inverse();
            Eigen::Matrix4d H2 = H_odo_cam * frameCurr->systemPose()->toMatrix().inverse();

            triangulatePoints(Eigen::Quaterniond(H1.block<3,3>(0,0)), Eigen::Vector3d(H1.block<3,1>(0,3)), imagePoints[0],
                              Eigen::Quaterniond(H2.block<3,3>(0,0)), Eigen::Vector3d(H2.block<3,1>(0,3)), imagePoints[1],
                              points3D, indices);
        }

        if (m_verbose)
        {
            std::cout << "# INFO: sliding windows : Triangulated " << points3D.size() << " points." << std::endl;
        }

        if ((int)points3D.size() < k_min2D3DFeatureCorrespondences)
        {
            if (m_verbose)
            {
                std::cout << "# INFO: Insufficient number of 2D-3D correspondences for BA initialization." << std::endl;
            }

            return false;
        }

        // 对于每个三维点
        for (size_t i = 0; i < points3D.size(); ++i)
        {
            size_t idx = indices.at(i);

            // 第i个三维点对应的二维像素点
            std::vector<Point2DFeaturePtr>& fc = inlierFeatureCorrespondences.at(idx);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);

            Point3DFeaturePtr point3D = boost::make_shared<Point3DFeature>();

            point3D->point() = points3D.at(i);

            // 三维点和二维像素点一一对应
            for (int j = 0; j < 2; ++j)
            {
                Point2DFeaturePtr& pt = fc.at(j);

                point3D->features2D().push_back(pt);
                pt->feature3D() = point3D;
            }

            f0->bestNextMatchId() = 0;
            f1->bestPrevMatchId() = 0;
        }
    }
    else // 如果不是初始的两帧
    {
        // find feature correspondences with associated 3D scene points
        /// 查找与关联三维场景点的特征对应关系

        std::vector<std::vector<Point2DFeaturePtr> > triFeatureCorrespondences;   // 存放的是匹配的2d点，其中点1存在对应的3维点
        std::vector<std::vector<Point2DFeaturePtr> > untriFeatureCorrespondences; // 存放的是匹配的2d点，其中点1没有对应的3维点
        for (size_t i = 0; i < featureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

            Point2DFeaturePtr& f0 = fc.at(0);
            //Point2DFeaturePtr& f1 = fc.at(1);

            if (f0->feature3D())
            {
                triFeatureCorrespondences.push_back(fc);
            }
            else
            {
                untriFeatureCorrespondences.push_back(fc);
            }
        }

        std::vector<size_t> inliers;
        if (m_mode == VO)
        {
            if ((int)triFeatureCorrespondences.size() < k_min2D3DFeatureCorrespondences)
            {
                if (m_verbose)
                {
                    std::cout << "# INFO: Insufficient number of 2D-3D correspondences (#" << triFeatureCorrespondences.size() << ") for P3P RANSAC." << std::endl;
                }

                return false;
            }

            if (m_verbose)
            {
                // 使用...个场景点通过P3P-RANSAC计算姿势。
                std::cout << "# INFO: Using " << triFeatureCorrespondences.size() << " scene points to compute pose via P3P RANSAC." << std::endl;
            }

            Eigen::Matrix4d H;
            // 通过P3P-RANSAC计算相机位姿
            solveP3PRansac(triFeatureCorrespondences, H, inliers);

            if (m_verbose)
            {
                std::cout << "# INFO: Sliding windows:(not initial frame:) Computed pose in frame " << m_frameCount - 1 << ":" << std::endl;

                std::cout <<"H :" << std::endl <<  H << std::endl;
            }

            // 当前帧相机的位姿赋值
            frameCurr->cameraPose()->rotation() = Eigen::Quaterniond(H.block<3,3>(0,0));
            frameCurr->cameraPose()->translation() = H.block<3,1>(0,3);
        }

        // remove feature correspondences marked as outliers in P3P RANSAC
        ///  删除P3P RANSAC中标记为异常值的特征对应

        std::vector<bool> inlierFlag(triFeatureCorrespondences.size(), false);
        for (size_t i = 0; i < inliers.size(); ++i)
        {
            // 最符合Ransac的内点的Vec的点赋值为true
            inlierFlag.at(inliers.at(i)) = true;
        }


        for (size_t i = 0; i < triFeatureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = triFeatureCorrespondences.at(i);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);

            if (inlierFlag.at(i))
            {
                // 点一和点2匹配，把点1对应的三维点赋值为点2对应的三维点
                f1->feature3D() = f0->feature3D();
                // 把三维点和点2对应起来
                f1->feature3D()->features2D().push_back(f1);

                f0->bestNextMatchId() = 0;
                f1->bestPrevMatchId() = 0;
            }
        }

        if (m_verbose) // 计算并输出重投影误差
        {
            size_t count = 0;
            double totalError = 0.0;

            for (size_t i = 0; i < inliers.size(); ++i)
            {
                const cv::Point2f& feature2D = triFeatureCorrespondences.at(i).at(1)->keypoint().pt;

                Eigen::Vector3d point3D = triFeatureCorrespondences.at(i).at(0)->feature3D()->point();

                double error;
                if (m_mode == VO)
                {
                    error = k_camera->reprojectionError(point3D,
                                                       frameCurr->cameraPose()->rotation(),
                                                       frameCurr->cameraPose()->translation(),
                                                       Eigen::Vector2d(feature2D.x, feature2D.y));
                }
                else
                {
                    error = reprojectionError(point3D,
                                              m_T_cam_odo.rotation(),
                                              m_T_cam_odo.translation(),
                                              frameCurr->systemPose()->position(),
                                              frameCurr->systemPose()->attitude(),
                                              Eigen::Vector2d(feature2D.x, feature2D.y));
                }

                totalError += error;
                ++count;
            }

            double avgError = totalError / count;

            printf("# INFO: sliding window : P3P RANSAC yielded %lu inliers with a reprojection error of %.2f px.\n", inliers.size(), avgError);
        }

        // use epipolar constraint to remove outlier 2D-2D correspondences
        /// 使用极线约束删除异常2D-2D对应

        // T_k,k-1 相机k-1帧到k帧的变换矩阵
        Eigen::Matrix4d H_rel = frameCurr->cameraPose()->toMatrix() * framePrev->cameraPose()->toMatrix().inverse();
        Eigen::Matrix3d R_rel = H_rel.block<3,3>(0,0);
        Eigen::Vector3d t_rel = H_rel.block<3,1>(0,3);
        // 本质矩阵
        Eigen::Matrix3d E = skew(t_rel) * R_rel;

        // 遍历没有三维匹配点的2d匹配点集合
        std::vector<std::vector<Point2DFeaturePtr> >::iterator it = untriFeatureCorrespondences.begin();
        while (it != untriFeatureCorrespondences.end())
        {
            cv::KeyPoint kpt1 = it->at(0)->keypoint();
            cv::KeyPoint kpt2 = it->at(1)->keypoint();

            Eigen::Vector3d spt1;
            // 把像素坐标系转换到相机坐标系
            k_camera->liftSphere(Eigen::Vector2d(kpt1.pt.x, kpt1.pt.y), spt1);

            Eigen::Vector3d spt2;
            // 把像素坐标系转换到相机坐标系
            k_camera->liftSphere(Eigen::Vector2d(kpt2.pt.x, kpt2.pt.y), spt2);

            double err = sampsonError(E, spt1, spt2);
            if (err < k_epipolarThresh)
            {
                ++it;
            }
            else
            {
                it = untriFeatureCorrespondences.erase(it);
            }
        }

        // triangulate new feature correspondences seen in last 2 frames
        /// 三角化最近2帧中看到的新特征对应(针对于没有三维点对应的匹配对集合)

        std::vector<cv::Point2f> ipoints[2];

        for (size_t i = 0; i < untriFeatureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = untriFeatureCorrespondences.at(i);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);

            ipoints[0].push_back(f0->keypoint().pt);
            ipoints[1].push_back(f1->keypoint().pt);
        }

        if (m_verbose)
        {
            std::cout << "# INFO: Sliding windows: Found " << untriFeatureCorrespondences.size() << " new!!! feature correspondences." << std::endl;
        }

        // 存在符合条件的 没有三维点对应的 匹配二维特征点对
        if (!untriFeatureCorrespondences.empty())
        {
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points3D;
            std::vector<size_t> indices;

            if (m_mode == VO)
            {
                triangulatePoints(framePrev->cameraPose()->rotation(), framePrev->cameraPose()->translation(), ipoints[0],
                                  frameCurr->cameraPose()->rotation(), frameCurr->cameraPose()->translation(), ipoints[1],
                                  points3D, indices);
            }
            else
            {
                Eigen::Matrix4d H_odo_cam = m_T_cam_odo.toMatrix().inverse();

                Eigen::Matrix4d H1 = H_odo_cam * framePrev->systemPose()->toMatrix().inverse();
                Eigen::Matrix4d H2 = H_odo_cam * frameCurr->systemPose()->toMatrix().inverse();

                triangulatePoints(Eigen::Quaterniond(H1.block<3,3>(0,0)), Eigen::Vector3d(H1.block<3,1>(0,3)), ipoints[0],
                                  Eigen::Quaterniond(H2.block<3,3>(0,0)), Eigen::Vector3d(H2.block<3,1>(0,3)), ipoints[1],
                                  points3D, indices);
            }

            if (m_verbose)
            {
                std::cout << "# INFO: Sliding windows: Triangulated " << points3D.size() << " new !!!!  points." << std::endl;
            }

            // 让三维点和二维点对匹配起来
            for (size_t i = 0; i < points3D.size(); ++i)
            {
                Point3DFeaturePtr point3D = boost::make_shared<Point3DFeature>();

                point3D->point() = points3D.at(i);

                std::vector<Point2DFeaturePtr>& fc = untriFeatureCorrespondences.at(indices.at(i));

                Point2DFeaturePtr& f0 = fc.at(0);
                Point2DFeaturePtr& f1 = fc.at(1);

                for (int j = 0; j < 2; ++j)
                {
                    Point2DFeaturePtr& pt = fc.at(j);

                    point3D->features2D().push_back(pt);
                    pt->feature3D() = point3D;
                }

                f0->bestNextMatchId() = 0;
                f1->bestPrevMatchId() = 0;
            }
        }
    } // 如果不是初始的两帧

    if (m_verbose)
    {
        double minError, maxError, avgError;

        windowReprojectionError(minError, maxError, avgError);
        // 优化前窗口重投影错误
        std::cout << "# INFO: Slinding Windows : Window reprojection error before optimization: min = " << minError << " | max = " << maxError << " | avg = " << avgError << std::endl;
    }

    bool runOptimization = false;

    // 遍历窗口中的帧
    for (std::list<FramePtr>::const_iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        const FrameConstPtr& frame = *it;

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point2DFeatureConstPtr& feature2D = features2D.at(i);

            // 如果窗口中帧的2D特征点有对应的三维点，就可以执行优化
            if (feature2D->feature3D())
            {
                runOptimization = true;
                break;
            }
        }

        if (runOptimization)
        {
            break;
        }
    }

    // perform BA to optimize camera poses and scene points
    ///执行BA以优化相机姿势和场景点

    if (runOptimization)
    {
        optimize();
    }

    // prune triangulated scene points with high reprojection error and behind a camera
    size_t nPrunedScenePoints = 0;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H_cam;
    H_cam.push_back(framePrev->cameraPose()->toMatrix());
    H_cam.push_back(frameCurr->cameraPose()->toMatrix());
    for (size_t i = 0; i < featureCorrespondences.size(); ++i)
    {
        std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

        bool prune = false;
        for (int j = 0; j < 2; ++j)
        {
            Point2DFeaturePtr& feature = fc.at(j);
            Point3DFeaturePtr& scenePoint = feature->feature3D();

            if (!scenePoint)
            {
                continue;
            }

            Eigen::Vector3d P = transformPoint(H_cam.at(j), scenePoint->point());

            if (P(2) < 0.0)
            {
                prune = true;
                break;
            }

            Eigen::Vector2d p;
            k_camera->spaceToPlane(P, p);

            double err = hypot(feature->keypoint().pt.x - p(0),
                               feature->keypoint().pt.y - p(1));

            if (err > k_reprojErrorThresh)
            {
                prune = true;
                break;
            }
        }

        if (prune)
        {
            Point2DFeaturePtr f0 = fc.at(0);
            Point2DFeaturePtr f1 = fc.at(1);
            Point3DFeaturePtr scenePoint = f0->feature3D();

            if (f0->prevMatches().empty() || f0->bestPrevMatchId() == -1)
            {
                f0->feature3D() = Point3DFeaturePtr();
                scenePoint->removeFeatureObservation(f0);
            }

            f1->feature3D() = Point3DFeaturePtr();
            scenePoint->removeFeatureObservation(f1);

            f0->bestNextMatchId() = -1;
            f1->bestPrevMatchId() = -1;

            ++nPrunedScenePoints;
        }
    }

    if (m_verbose)
    {
        if (nPrunedScenePoints > 0)
        {
            // 修剪。。。重投影错误太高的场景点。
            std::cout << "# INFO: Pruned " << nPrunedScenePoints << " scene points that had too high reprojection errors." << std::endl;
        }

        double minError, maxError, avgError;

        windowReprojectionError(minError, maxError, avgError);

        std::cout << "# INFO: Window reprojection error after optimization: min = " << minError << " | max = " << maxError << " | avg = " << avgError << std::endl;
    }

    return true;
}

void
SlidingWindowBA::clear(void)
{
    m_frameCount = 0;
    m_window.clear();
}

bool
SlidingWindowBA::empty(void) const
{
    return m_window.empty();
}

size_t
SlidingWindowBA::windowSize(void) const
{
    return m_window.size();
}

void
SlidingWindowBA::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

int
SlidingWindowBA::N(void)
{
    return m_N;
}

int
SlidingWindowBA::n(void)
{
    return m_n;
}

FramePtr&
SlidingWindowBA::currentFrame(void)
{
    return m_window.back();
}

// 获得相机的位姿
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > SlidingWindowBA::poses(void) const
{
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses;

    for (std::list<FramePtr>::const_iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        // 遍历每一帧
        const FrameConstPtr& frame = *it;

        Eigen::Matrix4d pose;
        pose.setIdentity();

        pose.block<3,3>(0,0) = frame->cameraPose()->rotation().toRotationMatrix();
        pose.block<3,1>(0,3) = frame->cameraPose()->translation();

        poses.push_back(pose);
    }

    return poses; // 返回相机的位姿
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >
SlidingWindowBA::scenePoints(void) const
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;

    boost::unordered_set<Point3DFeature*> set;

    for (std::list<FramePtr>::const_iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        const FrameConstPtr& frame = *it;

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point3DFeaturePtr& scenePoint = features2D.at(i)->feature3D();

            if (!scenePoint)
            {
                continue;
            }

            set.insert(scenePoint.get());
        }
    }

    for (boost::unordered_set<Point3DFeature*>::iterator it = set.begin(); it != set.end(); ++it)
    {
        scenePoints.push_back((*it)->point());
    }

    return scenePoints;
}

void
SlidingWindowBA::frameReprojectionError(int windowIdx, double& minError, double& maxError, double& avgError) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    std::list<FramePtr>::const_iterator it = m_window.begin();
    std::advance(it, windowIdx);

    const FrameConstPtr& frame = *it;

    const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

    for (size_t i = 0; i < features2D.size(); ++i)
    {
        const Point2DFeatureConstPtr& feature2D = features2D.at(i);
        const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

        if (!feature3D)
        {
            continue;
        }

        double error;
        if (m_mode == VO)
        {
            error = k_camera->reprojectionError(feature3D->point(),
                                               frame->cameraPose()->rotation(),
                                               frame->cameraPose()->translation(),
                                               Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));
        }
        else
        {
            error = reprojectionError(feature3D->point(),
                                      m_T_cam_odo.rotation(),
                                      m_T_cam_odo.translation(),
                                      frame->systemPose()->position(),
                                      frame->systemPose()->attitude(),
                                      Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));
        }

        if (minError > error)
        {
            minError = error;
        }
        if (maxError < error)
        {
            maxError = error;
        }
        totalError += error;
        ++count;
    }

    if (count == 0)
    {
        avgError = 0.0;
        minError = 0.0;
        maxError = 0.0;

        return;
    }

    avgError = totalError / count;
}

// 滑动窗口的重投影误差
void SlidingWindowBA::windowReprojectionError(double& minError, double& maxError, double& avgError) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    for (std::list<FramePtr>::const_iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        const FrameConstPtr& frame = *it;

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point2DFeatureConstPtr& feature2D = features2D.at(i);
            const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

            if (!feature3D)
            {
                continue;
            }

            double error;
            if (m_mode == VO)
            {
                error = k_camera->reprojectionError(feature3D->point(),
                                                   frame->cameraPose()->rotation(),
                                                   frame->cameraPose()->translation(),
                                                   Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));
            }
            else
            {
                error = reprojectionError(feature3D->point(),
                                          m_T_cam_odo.rotation(),
                                          m_T_cam_odo.translation(),
                                          frame->systemPose()->position(),
                                          frame->systemPose()->attitude(),
                                          Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));
            }

            if (minError > error)
            {
                minError = error;
            }
            if (maxError < error)
            {
                maxError = error;
            }
            totalError += error;
            ++count;
        }
    }

    if (count == 0)
    {
        avgError = 0.0;
        minError = 0.0;
        maxError = 0.0;

        return;
    }

    avgError = totalError / count;
}

double
SlidingWindowBA::reprojectionError(const Eigen::Vector3d& P,
                                   const Eigen::Quaterniond& cam_odo_q,
                                   const Eigen::Vector3d& cam_odo_t,
                                   const Eigen::Vector3d& odo_p,
                                   const Eigen::Vector3d& odo_att,
                                   const Eigen::Vector2d& observed_p) const
{
    Eigen::Quaterniond q_z(cos(odo_att(0) / 2.0), 0.0, 0.0, sin(odo_att(0) / 2.0));
    Eigen::Quaterniond q_y(cos(odo_att(1) / 2.0), 0.0, sin(odo_att(1) / 2.0), 0.0);
    Eigen::Quaterniond q_x(cos(odo_att(2) / 2.0), sin(odo_att(2) / 2.0), 0.0, 0.0);

    Eigen::Quaterniond odo_q = q_z * q_y * q_x;
    Eigen::Vector3d odo_t = odo_p;

    Eigen::Quaterniond cam_q = cam_odo_q.conjugate() * odo_q.conjugate();
    Eigen::Vector3d cam_t = -cam_odo_q.conjugate().toRotationMatrix() * (-odo_q.conjugate().toRotationMatrix() * odo_t);

    return k_camera->reprojectionError(P, cam_q, cam_t, observed_p);
}
/**
 * 把满足要求的匹配点 存放到 2d匹配特征点的集合
 * @param features  当前帧的特征点
 * @param nViews
 * @param correspondences 2d匹配特征点的集合
 */
void SlidingWindowBA::findFeatureCorrespondences(const std::vector<Point2DFeaturePtr>& features,
                                            int nViews,
                                            std::vector<std::vector<Point2DFeaturePtr> >& correspondences) const
{
    // find feature correspondences across n views starting backward from specified feature set in nth view
    /// 从第n个视图中指定的特征集向后开始，在n个视图中查找特征对应关系

    if (nViews < 2)
    {
        return;
    }

    correspondences.reserve(features.size());

    // 遍历所有的特征点
    for (size_t i = 0; i < features.size(); ++i)
    {
        std::vector<Point2DFeaturePtr> pt(nViews);

        pt[nViews - 1] = features.at(i);
        bool foundCorrespondences = true;

        for (int j = nViews - 1; j > 0; --j)
        {
            // 如果该特征点相对于上一帧的匹配为空，或者没有最佳匹配，则没找到匹配点
            if (pt[j]->prevMatches().empty() || pt[j]->bestPrevMatchId() == -1)
            {
                foundCorrespondences = false;
                break;
            }

            // pt[j-1] = 特征点在上一帧的匹配
            pt[j - 1] = pt[j]->prevMatch().lock();

            if (!pt[j - 1])
            {
                foundCorrespondences = false;
                break;
            }
        }

        if (!foundCorrespondences)
        {
            continue;
        }

        bool rejectCorrespondence = false;
        for (int j = 0; j < nViews - 1; ++j)
        {
            // 前后帧匹配关键点的欧式距离比最小阈值还要小，则拒绝
            if (cv::norm(pt[j]->keypoint().pt - pt[j + 1]->keypoint().pt) < k_minDisparity)
            {
                rejectCorrespondence = true;
                break;
            }
        }

        if (rejectCorrespondence)
        {
            continue;
        }

        std::vector<Point2DFeaturePtr> correspondence(nViews);
        for (int j = 0; j < nViews; ++j)
        {
            correspondence.at(j) = pt[j];
        }

        // 把满足要求的匹配点 存放到 2d匹配特征点的集合
        correspondences.push_back(correspondence);
    }
}

/**
 * 通过P3P-RANSAC计算姿势
 * @param correspondences
 * @param H 计算获得的相机位姿
 * @param inliers 最符合Ransac的内点的Vec
 */
void SlidingWindowBA::solveP3PRansac(const std::vector<std::vector<Point2DFeaturePtr> >& correspondences,
                                Eigen::Matrix4d& H,
                                std::vector<size_t>& inliers) const
{
    inliers.clear();

    double p = 0.99; // probability that at least one set of random samples does not contain an outlier
                     // 至少一组随机样本不包含异常值的概率
    double v = 0.6; // probability of observing an outlier
                    // 观察异常值的概率

    double u = 1.0 - v;
    int N = static_cast<int>(log(1.0 - p) / log(1.0 - u * u * u) + 0.5);

    std::vector<size_t> indices;
    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        indices.push_back(i);
    }

    // run RANSAC to find best H
    Eigen::Matrix4d H_best;
    std::vector<size_t> inlierIds_best;
    for (int i = 0; i < N; ++i)
    {
        std::random_shuffle(indices.begin(), indices.end());

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rays(3);
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > worldPoints(3);
        for (int j = 0; j < 3; ++j)
        {
            const std::vector<Point2DFeaturePtr>& corr = correspondences.at(indices.at(j));

            // 三维点
            worldPoints.at(j) = corr.at(0)->feature3D()->point();

            const cv::KeyPoint& kpt2 = corr.at(1)->keypoint();

            Eigen::Vector3d ray;
            // 把像素坐标系转换到相机坐标系
            k_camera->liftSphere(Eigen::Vector2d(kpt2.pt.x, kpt2.pt.y), ray);

            rays.at(j) = ray;
        }

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > solutions;
        if (!solveP3P(rays, worldPoints, solutions))
        {
            continue;
        }

        for (size_t j = 0; j < solutions.size(); ++j)
        {
            Eigen::Matrix4d H_inv = solutions.at(j).inverse();

            std::vector<size_t> inliersIds;
            // 计算重投影误差，删去误差过大的点
            for (size_t k = 0; k < correspondences.size(); ++k)
            {
                const std::vector<Point2DFeaturePtr>& corr = correspondences.at(k);

                Eigen::Vector3d P1 = corr.at(0)->feature3D()->point();

                Eigen::Vector3d P2 = transformPoint(H_inv, P1);
                Eigen::Vector2d p2_pred;
                // 把三维点投影到二维平面
                k_camera->spaceToPlane(P2, p2_pred);

                const Point2DFeatureConstPtr& f2 = corr.at(1);

                double err = hypot(f2->keypoint().pt.x - p2_pred(0),
                                   f2->keypoint().pt.y - p2_pred(1));
                if (err > k_reprojErrorThresh)
                {
                    continue;
                }

                inliersIds.push_back(k);
            }

            if (inliersIds.size() > inlierIds_best.size())
            {
                H_best = H_inv;
                inlierIds_best = inliersIds;
            }
        }
    }

    H = H_best;
    inliers = inlierIds_best;
}

bool
SlidingWindowBA::project3DPoint(const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
                                const Eigen::Vector3d& src, Eigen::Vector2d& dst) const
{
    // transform point from world frame to camera frame
    Eigen::Vector3d P = q.toRotationMatrix() * src + t;

    // check if point is behind camera
    if (P(2) < 0.0)
    {
        return false;
    }

    Eigen::Vector2d p;
    k_camera->spaceToPlane(P, p);
    dst << p;

    return true;
}

/**
 * 三角化特征点
 * @param q1  上一帧相机的旋转
 * @param t1  上一帧相机的平移
 * @param imagePoints1  上一帧相机的图像点
 * @param q2  当前帧相机的旋转
 * @param t2  当前帧相机的平移
 * @param imagePoints2  当前帧相机的图像点
 * @param points3D  三维点
 * @param inliers   存放内点id的vector
 */
void SlidingWindowBA::triangulatePoints(const Eigen::Quaterniond& q1,
                                   const Eigen::Vector3d& t1,
                                   const std::vector<cv::Point2f>& imagePoints1,
                                   const Eigen::Quaterniond& q2,
                                   const Eigen::Vector3d& t2,
                                   const std::vector<cv::Point2f>& imagePoints2,
                                   std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points3D,
                                   std::vector<size_t>& inliers) const
{
    // 相机1的 Tcw
    Eigen::Matrix4d H_cam1 = homogeneousTransform(q1.toRotationMatrix(), t1);
    // 相机1的 Twc
    Eigen::Matrix4d H_cam1_inv = H_cam1.inverse();
    // 相机2的 Tcw
    Eigen::Matrix4d H_cam2 = homogeneousTransform(q2.toRotationMatrix(), t2);
    Eigen::Matrix4d H = H_cam2 * H_cam1_inv;

    for (size_t i = 0; i < imagePoints1.size(); ++i)
    {
        const cv::Point2f& p1_cv = imagePoints1.at(i);
        const cv::Point2f& p2_cv = imagePoints2.at(i);

        Eigen::Vector3d spt1;
        // 把像素坐标系转换到相机坐标系
        k_camera->liftSphere(Eigen::Vector2d(p1_cv.x, p1_cv.y), spt1);

        Eigen::Vector3d spt2;
        // 把像素坐标系转换到相机坐标系
        k_camera->liftSphere(Eigen::Vector2d(p2_cv.x, p2_cv.y), spt2);

        Eigen::MatrixXd A(3,2);
        A.col(0) = H.block<3,3>(0,0) * spt1;
        A.col(1) = - spt2;

        Eigen::Vector3d b = - H.block<3,1>(0,3);

        Eigen::Vector2d gamma = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        // check if scene point is behind camera
        if (gamma(0) < 0.0 || gamma(1) < 0.0)
        {
            continue;
        }

        Eigen::Vector3d P = gamma(0) * spt1;
        // 把三维点转换到相机坐标系
        P = transformPoint(H_cam1_inv, P);

        // validate scene point 验证场景点是否有效
        Eigen::Vector2d p2;
        if (!project3DPoint(q2, t2, P.block<3,1>(0,0), p2))
        {
            continue;
        }

        if (m_mode == VO)
        {
            if ((p2 - Eigen::Vector2d(p2_cv.x, p2_cv.y)).norm() > k_reprojErrorThresh)
            {
                continue;
            }
        }

        points3D.push_back(P);
        inliers.push_back(i); // 存放内点的id（也对应了每个三维点id）
    }
}

// 优化
void SlidingWindowBA::optimize(void)
{
    ceres::Problem problem;

    ceres::Solver::Options options;
    // 对于大规模稀疏矩阵，增量方式用DENSE_SCHUR
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // 最大迭代次数
    options.max_num_iterations = 20;

    for (std::list<FramePtr>::iterator it = m_window.begin(); it != m_window.end(); ++it)
    {
        // 遍历滑动窗口中的帧
        FramePtr& frame = *it;

        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        bool optimizeFrame = false;
        for (size_t i = 0; i < features2D.size(); ++i) // 遍历该帧的每一个2D点
        {
            // 遍历该图像帧的每个2D特征点
            Point2DFeaturePtr& feature2D = features2D.at(i);

            // 如果2D特征点没有3D点，则跳过
            if (!feature2D->feature3D())
            {
                continue;
            }

            // 核函数
            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);

            if (m_mode == VO)// m_mode 就是VO
            {
                // 构建costFunction：
                ceres::CostFunction* costFunction =
                    CostFunctionFactory::instance()->generateCostFunction(k_camera,
                                                                          Eigen::Vector2d(feature2D->keypoint().pt.x,
                                                                                          feature2D->keypoint().pt.y),
                                                                          CAMERA_POSE | POINT_3D);

                // 添加残差项，优化： 1.该帧相机位姿的旋转，2.该帧相机位姿的平移，3.2D特征点对应的三维点
                problem.AddResidualBlock(costFunction, lossFunction,
                                         frame->cameraPose()->rotationData(), frame->cameraPose()->translationData(),
                                         feature2D->feature3D()->pointData());
            }
            else // m_mode == ODOMETRY
            {
                ceres::CostFunction* costFunction =
                    CostFunctionFactory::instance()->generateCostFunction(k_camera,
                                                                          Eigen::Vector2d(feature2D->keypoint().pt.x,
                                                                                          feature2D->keypoint().pt.y),
                                                                          CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_3D_POSE | POINT_3D);

                // 添加误差项： 优化：1.外参的旋转，2.外参的平移，3 该帧的里程计位姿.4 该帧的里程计的欧拉角 5.2D特征点对应的三维点
                problem.AddResidualBlock(costFunction, lossFunction,
                                         m_T_cam_odo.rotationData(),
                                         m_T_cam_odo.translationData(),
                                         frame->systemPose()->positionData(),
                                         frame->systemPose()->attitudeData(),
                                         feature2D->feature3D()->pointData());
            }

            optimizeFrame = true;
        }

        if (optimizeFrame)
        {
            if (m_mode == VO)
            {
                // 四元数参数化
                ceres::LocalParameterization* quaternionParameterization =
                    new ceres::QuaternionParameterization;

                // 为参数块(frame->cameraPose()->rotationData())设置parameter(quaternionParameterization)
                problem.SetParameterization(frame->cameraPose()->rotationData(), quaternionParameterization);
            }
            else
            {
                // 在优化过程中保持指定的参数块不变
                problem.SetParameterBlockConstant(frame->systemPose()->positionData());
                problem.SetParameterBlockConstant(frame->systemPose()->attitudeData());
            }
        }
    }

    if (m_mode == ODOMETRY)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new ceres::QuaternionParameterization;

        // 为参数块(m_T_cam_odo.rotationData())设置parameter(quaternionParameterization,变为四元数)
        problem.SetParameterization(m_T_cam_odo.rotationData(), quaternionParameterization);
    }

    if ((int)m_window.size() > m_N - m_n) // 如果滑窗中帧的数量大于14
    {
        std::list<FramePtr>::iterator it = m_window.begin();
        for (int i = 0; i < m_N - m_n; ++i) // 固定住一定数量的相机位姿
        {
            FramePtr& frame = *it;

            if (m_mode == VO) // 如果是VO模式，就固定住相机的旋转和平移
            {
                // SetParameterBlockConstant: 在优化过程中保持指定的参数块不变。
                // 优化过程中固定住相机的旋转和平移
                problem.SetParameterBlockConstant(frame->cameraPose()->rotationData());
                problem.SetParameterBlockConstant(frame->cameraPose()->translationData());
            }

            ++it;
        }
    }
    else // 如果滑窗中帧的数量<= 14
    {
        std::list<FramePtr>::iterator it = m_window.begin();
        for (int i = 0; i < 1; ++i) // 只固定第一帧的相机位姿
        {
            FramePtr& frame = *it;

            // 如果是VO模式，就固定第一帧的相机位姿
            if (m_mode == VO)
            {
                // SetParameterBlockConstant: 在优化过程中保持指定的参数块不变。
                // 优化过程中固定住相机的旋转和平移
                problem.SetParameterBlockConstant(frame->cameraPose()->rotationData());
                problem.SetParameterBlockConstant(frame->cameraPose()->translationData());
            }

            ++it;
        }
    }

    ceres::Solver::Summary summary;
    // 开始优化
    ceres::Solve(options, &problem, &summary);
    std::cout << "Sling windows BA:" << std::endl << summary.BriefReport() << std::endl;
}

}
