#ifndef SLIDINGWINDOWBA_H
#define SLIDINGWINDOWBA_H

#include <Eigen/Dense>
#include <list>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

class SlidingWindowBA
{
public:
    enum
    {
        ODOMETRY = 0,
        VO = 1
    };

    SlidingWindowBA(const CameraConstPtr& camera,
                    int N = 20, int n = 6, int mode = VO,
                    Eigen::Matrix4d globalCameraPose = Eigen::Matrix4d());

    Eigen::Matrix4d globalCameraPose(void);

    bool addFrame(FramePtr& frame);

    void clear(void);
    bool empty(void) const;
    size_t windowSize(void) const;

    void setVerbose(bool verbose);

    int N(void);
    int n(void);

    FramePtr& currentFrame(void);
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses(void) const;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints(void) const;

    void frameReprojectionError(int windowIdx, double& minError, double& maxError, double& avgError) const;
    void windowReprojectionError(double& minError, double& maxError, double& avgError) const;

private:
    double reprojectionError(const Eigen::Vector3d& P,
                             const Eigen::Quaterniond& cam_odo_q,
                             const Eigen::Vector3d& cam_odo_t,
                             const Eigen::Vector3d& odo_p,
                             const Eigen::Vector3d& odo_att,
                             const Eigen::Vector2d& observed_p) const;

    void findFeatureCorrespondences(const std::vector<Point2DFeaturePtr>& features,
                                    int nViews,
                                    std::vector<std::vector<Point2DFeaturePtr> >& correspondences) const;

    void solveP3PRansac(const std::vector<std::vector<Point2DFeaturePtr> >& correspondences,
                        Eigen::Matrix4d& H,
                        std::vector<size_t>& inliers) const;

    bool project3DPoint(const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
                        const Eigen::Vector3d& src, Eigen::Vector2d& dst) const;

    void triangulatePoints(const Eigen::Quaterniond& q1,
                           const Eigen::Vector3d& t1,
                           const std::vector<cv::Point2f>& imagePoints1,
                           const Eigen::Quaterniond& q2,
                           const Eigen::Vector3d& t2,
                           const std::vector<cv::Point2f>& imagePoints2,
                           std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points3D,
                           std::vector<size_t>& inliers) const;

    void optimize(void);

    int m_N;        // 滑窗设定最大size
    int m_n;
    int m_mode;

    Pose m_T_cam_odo;

    std::list<FramePtr> m_window; // 滑动窗口

    const CameraConstPtr k_camera;

    const double k_epipolarThresh;
    const double k_minDisparity;
    const double k_nominalFocalLength;
    const double k_reprojErrorThresh;

    size_t m_frameCount; // 添加帧的总数
    bool m_verbose;

    const int k_min2D2DFeatureCorrespondences;  // 用于BA初始化的2D-2D的最小特征点对应数量
    const int k_min2D3DFeatureCorrespondences;  // 用于BA初始化的2D-3D的最小特征点对应数量
};

}

#endif
