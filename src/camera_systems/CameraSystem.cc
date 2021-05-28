#include <camodocal/camera_systems/CameraSystem.h>

#include <boost/filesystem.hpp>
#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <fstream>
#include <iomanip>

#include "../pugixml/pugixml.hpp"

namespace camodocal
{

CameraSystem::CameraSystem()
 : m_cameraCount(0)
 , m_referenceCameraIdx(-1)
{

}

CameraSystem::CameraSystem(int cameraCount)
 : m_cameraCount(cameraCount)
 , m_referenceCameraIdx(-1)
{
    m_cameras.resize(cameraCount);
    m_globalPoses.resize(cameraCount, Eigen::Matrix4d::Identity());
}

int
CameraSystem::cameraCount(void) const
{
    return m_cameraCount;
}

void
CameraSystem::reset(void)
{
    for (int i = 0; i < m_cameraCount; ++i)
    {
        m_cameras.at(i).reset();
        m_globalPoses.at(i).setIdentity();
    }
}


bool
CameraSystem::writePosesToTextFile(const std::string& filename) const
{
    std::ofstream ofs(filename.c_str());

    if (!ofs.is_open())
    {
        return false;
    }

    ofs << std::fixed << std::setprecision(10);

    for (int i = 0; i < m_cameraCount; ++i)
    {
        const Eigen::Matrix4d& globalPose = m_globalPoses.at(i);

        if (m_cameras.at(i)->cameraName().empty())
        {
            ofs << "camera_" << i;
        }
        else
        {
            ofs << m_cameras.at(i)->cameraName();
        }
        ofs << std::endl;

        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                ofs << globalPose(j,k);

                if (k < 3)
                {
                    ofs << " ";
                }
            }
            ofs << std::endl;
        }
        ofs << std::endl;
    }

    ofs.close();

    return true;
}


bool
CameraSystem::writeToDirectory(const std::string& directory) const
{
    if (!boost::filesystem::is_directory(directory))
    {
        boost::filesystem::create_directory(directory);
    }

    // write extrinsic data
    boost::filesystem::path extrinsicPath(directory);
    extrinsicPath /= "extrinsic.txt";

    writePosesToTextFile(extrinsicPath.string());

    // write intrinsic data
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        std::ostringstream oss;
        if (m_cameras.at(i)->cameraName().empty())
        {
            oss << "camera_" << i << "_calib.yaml";
        }
        else
        {
            oss << m_cameras.at(i)->cameraName() << "_calib.yaml";
        }

        boost::filesystem::path intrinsicPath(directory);
        intrinsicPath /= oss.str();

        m_cameras.at(i)->writeParametersToYamlFile(intrinsicPath.string());
    }

    return true;
}




int
CameraSystem::getCameraIdx(const CameraConstPtr& camera) const
{
    CameraPtr cameraP = boost::const_pointer_cast<Camera>(camera);

    boost::unordered_map<CameraPtr,int>::const_iterator it = m_cameraMap.find(cameraP);
    if (it == m_cameraMap.end())
    {
        return -1;
    }
    else
    {
        return it->second;
    }
}

CameraPtr
CameraSystem::getCamera(int idx) const
{
    return m_cameras.at(idx);
}

bool CameraSystem::setCamera(int idx, CameraPtr& camera)
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return false;
    }

    m_cameras.at(idx) = camera;

    boost::unordered_map<CameraPtr,int>::iterator it = m_cameraMap.begin();
    while (it != m_cameraMap.end())
    {
        if (it->second == idx)
        {
            m_cameraMap.erase(it);
        }
        else
        {
            ++it;
        }
    }

    m_cameraMap.insert(std::make_pair(camera, idx));

    return true;
}

bool
CameraSystem::setReferenceCamera(int idx)
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return false;
    }

    m_referenceCameraIdx = idx;

    return true;
}

Eigen::Matrix4d
CameraSystem::getGlobalCameraPose(int idx) const
{
    return m_globalPoses.at(idx);
}

Eigen::Matrix4d
CameraSystem::getGlobalCameraPose(const CameraConstPtr& camera) const
{
    int idx = getCameraIdx(camera);
    if (idx == -1)
    {
        return Eigen::Matrix4d::Zero();
    }

    return m_globalPoses.at(idx);
}

bool
CameraSystem::setGlobalCameraPose(int idx, const Eigen::Matrix4d& pose)
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return false;
    }

    m_globalPoses.at(idx) = pose; // 每一个相机赋值世界坐标系下位姿

    return true;
}


CameraSystem&
CameraSystem::operator=(const CameraSystem& other)
{
    if (this != &other) // protect against invalid self-assignment
    {
        m_cameraCount = other.m_cameraCount;
        m_referenceCameraIdx = other.m_referenceCameraIdx;
        m_cameras = other.m_cameras;
        m_globalPoses = other.m_globalPoses;
    }

    return *this;
}

}
