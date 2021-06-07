#ifndef CAMODOCALIBRATION_H
#define CAMODOCALIBRATION_H

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>

namespace camodocal
{

class CamOdoCalibration
{
public:
    CamOdoCalibration();

    bool addMotionSegment(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_cam,
                          const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_odo);


    size_t getCurrentMotionCount(void) const;
    size_t getMotionCount(void) const;
    void setMotionCount(size_t count);

    bool calibrate(Eigen::Matrix4d& H_cam_odo);


    bool getVerbose(void);
    void setVerbose(bool on = false);

    bool estimate(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                  const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2,
                  Eigen::Matrix4d& H_cam_odo,
                  std::vector<double>& scales) const;

private:
    bool estimateRyx(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                     const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                     const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                     const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2,
                     Eigen::Matrix3d& R_yx) const;
    
    void refineEstimate(Eigen::Matrix4d& H_cam_odo, std::vector<double>& scales,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs1,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs1,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& rvecs2,
                        const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& tvecs2) const;

    // solve ax^2 + bx + c = 0
    bool solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const;

    typedef struct
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d rotation;
        Eigen::Vector3d translation;
    } Motion;

    typedef struct
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<Motion, Eigen::aligned_allocator<Motion> > odoMotions;
        std::vector<Motion, Eigen::aligned_allocator<Motion> > camMotions;
    } MotionSegment;

    /// 存储每一个MotionSegment，MotionSegment为每次跟踪失败或停止前每一个相机帧对应的相机帧间位姿和里程计位姿
    std::vector<MotionSegment> mSegments;

    size_t mMinMotions;

    bool mVerbose;
};

}

#endif
