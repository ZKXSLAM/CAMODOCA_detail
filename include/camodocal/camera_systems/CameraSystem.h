#ifndef CAMERASYSTEM_H
#define CAMERASYSTEM_H

#include <boost/unordered_map.hpp>
#include <camodocal/camera_models/Camera.h>
#include <Eigen/Eigen>
#include <vector>

namespace camodocal
{

class CameraSystem
{
public:
    CameraSystem();
    CameraSystem(int cameraCount);

    int cameraCount(void) const;

    void reset(void);

    bool writePosesToTextFile(const std::string& filename) const;

    bool writeToDirectory(const std::string& directory) const;

    int getCameraIdx(const CameraConstPtr& camera) const;
    CameraPtr getCamera(int idx) const;

    bool setCamera(int idx, CameraPtr& camera);

    bool setReferenceCamera(int idx);

    // global camera pose is the transform from camera frame to system's reference frame
    // 全局相机姿态是从相机帧到系统参考帧的变换 Tcb
    Eigen::Matrix4d getGlobalCameraPose(int idx) const;
    Eigen::Matrix4d getGlobalCameraPose(const CameraConstPtr& camera) const;


    bool setGlobalCameraPose(int idx, const Eigen::Matrix4d& pose);

    CameraSystem& operator=(const CameraSystem& other);

private:
    int m_cameraCount;
    int m_referenceCameraIdx;

    std::vector<CameraPtr> m_cameras;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > m_globalPoses;
    boost::unordered_map<CameraPtr,int> m_cameraMap;
};

typedef boost::shared_ptr<CameraSystem> CameraSystemPtr;
typedef boost::shared_ptr<const CameraSystem> CameraSystemConstPtr;

}

#endif
