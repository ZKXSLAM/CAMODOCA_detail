#ifndef CAMRIGODOCALIBRATION_H
#define CAMRIGODOCALIBRATION_H

#include <boost/asio.hpp>
#include <boost/multi_array.hpp>
#include "camodocal/calib/AtomicData.h"
#include "camodocal/calib/PoseSource.h"
#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/camera_systems/CameraSystem.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

// forward declarations
class CamOdoThread;
class CamOdoWatchdogThread;

class CamRigOdoCalibration
{
public:
    enum Mode
    {
        OFFLINE,
        ONLINE
    };

    class Options
    {
    public:
        Options()
         : mode(OFFLINE)
         , poseSource(ODOMETRY)
         , nMotions(200)
         , minKeyframeDistance(0.2)
         , minVOSegmentSize(15)
         , windowDistance(3.0)
         , preprocessImages(false)
         , saveWorkingData(true)
         , beginStage(0)
         , optimizeIntrinsics(false)
         , verbose(false) {};

        Mode mode;               // offline
        PoseSource poseSource;   // 位姿来源 ODOMETRY;
        int nMotions;            // Once we reach a number of keyframes for each camera such that there are <nMotion> relative
                                 // motions between consecutive keyframes, the calibration runs automatically.
                                 // 一旦我们获得每个摄影机的一定数量的关键帧，使得连续关键帧之间存在<nMotion>相对运动，校准将自动运行
                                 // 200,可设置

        // monocular VO
        double minKeyframeDistance; // Minimum distance between consecutive keyframes. 连续关键帧之间的最小距离。
                                    // (Recommended: 0.2 m) 可设置

        size_t minVOSegmentSize;    // The VO segment will be used in calibration only if the number of
                                    // keyframes in the VO segment exceeds <minVOSegmentSize>.
                                    // 仅当VOsegment中的关键帧数超过<minVOSegmentSize>时，才会在校准中使用VOsegment。
                                    // 15

        // local matching between cameras 摄像机之间的局部匹配
        double windowDistance;   // The size of the window of frames in which local matching is performed between different cameras depends on the
                                 // <windowDistance> distance that the vehicle travels from the beginning of the window to the end of the window.
                                 // The larger the distance, the longer the local matching takes.
                                 // 在不同相机之间执行局部匹配的帧窗口的大小取决于车辆从窗口的开始到窗口末端的<windowDistance>距离。距离越大，局部匹配所需时间越长。
                                 // 3

        bool preprocessImages;  // false
        bool saveWorkingData;  // 是否保存数据 true
        int beginStage;        // 开始阶段（帧） 0
        bool optimizeIntrinsics;  // 是否优化内参  false，可设置
        std::string dataDir;   // 保存工作数据的地址 （/data）
        bool verbose;          // 缓存,显示额外信息
    };

    CamRigOdoCalibration(std::vector<CameraPtr>& cameras,
                         const Options& options);
    virtual ~CamRigOdoCalibration();

    void addFrame(int cameraIdx, const cv::Mat& image, uint64_t timestamp);
    void addFrameSet(const std::vector<cv::Mat>& images, uint64_t timestamp);

    void addOdometry(double x, double y, double yaw, uint64_t timestamp) { addOdometry(x,y,0,yaw,timestamp); }
    void addOdometry(double x, double y, double z, double yaw, uint64_t timestamp);

    void addGpsIns(double lat, double lon, double alt,
                   double roll, double pitch, double yaw,
                   uint64_t timestamp);
    void addGpsIns(double lat, double lon, double alt,
                   double qx, double qy, double qz, double qw,
                   uint64_t timestamp);

    //! If an initial odo transform estimate for a camera is specified there will be no automatic estimation step performed. (@note setup before start()!)
    // 如果指定了相机的初始odo变换估计，则不会执行自动估计步骤(@开始前注意设置（）！）
    void setInitialCameraOdoTransformEstimates(unsigned camIdx, const Eigen::Matrix4d& odoT);

    void start(void);
    void run(void);

    bool isRunning(void) const;

    const CameraSystem& cameraSystem(void) const;

private:
    void onCamOdoThreadFinished(CamOdoThread* odoCamThread);

    void buildGraph(void);

    void pollWindow(boost::asio::io_service* io, bool* stop);
    void displayHandler(boost::asio::deadline_timer* timer, bool* stop);

    std::vector<CamOdoThread*> m_camOdoThreads;   // 存储每个CamOdoThread线程指针的Vec
    CamOdoWatchdogThread* m_camOdoWatchdogThread;

    CameraSystem m_cameraSystem;
    SparseGraph m_graph;

    std::vector<AtomicData<cv::Mat>* > m_images;
    std::vector<CameraPtr> m_cameras;               // 存储多个相机指针的Vec
    SensorDataBuffer<OdometryPtr> m_odometryBuffer;
    SensorDataBuffer<OdometryPtr> m_interpOdometryBuffer;
    boost::mutex m_odometryBufferMutex;
    SensorDataBuffer<PosePtr> m_gpsInsBuffer;
    SensorDataBuffer<PosePtr> m_interpGpsInsBuffer;
    boost::mutex m_gpsInsBufferMutex;

    boost::multi_array<bool, 1> m_camOdoCompleted;

    std::vector<cv::Mat> m_sketches;

    Options m_options;

    bool m_running;
    static bool m_stop;
};

}

#endif
