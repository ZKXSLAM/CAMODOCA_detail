#ifndef CAMODOTHREAD_H
#define CAMODOTHREAD_H

#include <boost/thread.hpp>
#include <boost/signals2.hpp>

#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/CamOdoCalibration.h"
#include "camodocal/calib/PoseSource.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_models/Camera.h"
#include "camodocal/sparse_graph/SparseGraph.h"

#ifdef VCHARGE_VIZ
#include "../../../../visualization/overlay/GLOverlayExtended.h"
#include "../visual_odometry/FeatureTracker.h"
#endif

namespace camodocal
{

class CamOdoThread
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CamOdoThread(PoseSource poseSource,  // PoseSource::GPS_INS : PoseSource::ODOMETRY; 位姿来源
                 int nMotions,           // 关键帧数目
                 int cameraId,           // 第几个相机
                 bool preprocess,        // 是否（有？）预处理图片
                 AtomicData<cv::Mat>* image,
                 const CameraConstPtr& camera,  // 相机指针
                 SensorDataBuffer<OdometryPtr>& odometryBuffer,  //里程计数据缓存器
                 SensorDataBuffer<OdometryPtr>& interpOdometryBuffer,
                 boost::mutex& odometryBufferMutex,
                 SensorDataBuffer<PosePtr>& gpsInsBuffer,
                 SensorDataBuffer<PosePtr>& interpGpsInsBuffer,
                 boost::mutex& gpsInsBufferMutex,
                 cv::Mat& sketch,
                 bool& completed,
                 bool& stop,
                 double minKeyframeDistance,    // 最小关键帧距离
                 size_t minVOSegmentSize,       // 最小VO segmen大小
                 bool verbose = false);
    virtual ~CamOdoThread();

    int cameraId(void) const;
    const Eigen::Matrix4d& camOdoTransform(void) const;
    const std::vector<std::vector<FramePtr> >& frameSegments(void) const;

    void setCamOdoTransformEstimate(const Eigen::Matrix4d& estimate);
    void clearCamOdoTransformEstimate();
    void reprojectionError(double& minError, double& maxError, double& avgError) const;

    void launch(void);
    void join(void);
    bool running(void) const;
    boost::signals2::signal<void ()>& signalFinished(void);

private:
    void threadFunction(void);

    void addCamOdoCalibData(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                            const std::vector<OdometryPtr>& odoPoses,
                            std::vector<FramePtr>& frameSegment);

#ifdef VCHARGE_VIZ
    void visualizeMap(vcharge::GLOverlayExtended& overlay,
                      TemporalFeatureTracker& tracker);
#endif

    PoseSource m_poseSource;  // m_poseSource == ODOMETRY

    boost::shared_ptr<boost::thread> m_thread;
    int m_cameraId;                 // 第几个相机
    bool m_preprocess;              // 是否预处理图片: false
    volatile bool m_running; // poor man's synchronisation
    boost::signals2::signal<void ()> m_signalFinished;

    CamOdoCalibration m_camOdoCalib;
    std::vector<std::vector<FramePtr> > m_frameSegments; // 存储滑动窗口的每一帧

    AtomicData<cv::Mat>* m_image;    // 某一帧的图像
    const CameraConstPtr m_camera;   // 相机指针
    SensorDataBuffer<OdometryPtr>& m_odometryBuffer;
    SensorDataBuffer<OdometryPtr>& m_interpOdometryBuffer;  // 存储插值后的里程计位姿
    boost::mutex& m_odometryBufferMutex;  // 锁
    SensorDataBuffer<PosePtr>& m_gpsInsBuffer;
    SensorDataBuffer<PosePtr>& m_interpGpsInsBuffer;
    boost::mutex& m_gpsInsBufferMutex;
    Eigen::Matrix4d m_camOdoTransform;
    bool m_camOdoTransformUseEstimate;
    cv::Mat& m_sketch;

    bool& m_completed;
    bool& m_stop;

    const double k_minKeyframeDistance;   // 最小关键帧距离
    const size_t k_minVOSegmentSize;      // 最小VO segment大小 15
    const double k_odometryTimeout;
};

}

#endif
