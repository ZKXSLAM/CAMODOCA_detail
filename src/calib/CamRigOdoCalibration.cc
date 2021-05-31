#include "camodocal/calib/CamRigOdoCalibration.h"

#include <boost/filesystem.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "../../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#include "CamOdoThread.h"
#include "CamOdoWatchdogThread.h"
#include "CameraRigBA.h"
#ifdef VCHARGE_VIZ
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../../../visualization/overlay/GLOverlayExtended.h"
#endif

namespace camodocal
{

bool CamRigOdoCalibration::m_stop = false;

CamRigOdoCalibration::CamRigOdoCalibration(std::vector<CameraPtr>& cameras,
                                           const Options& options)
 : m_camOdoThreads(cameras.size())
 , m_cameraSystem(cameras.size())
 , m_images(cameras.size())
 , m_cameras(cameras)
 , m_odometryBuffer(1000)
 , m_gpsInsBuffer(1000)
 , m_camOdoCompleted(boost::extents[cameras.size()])
 , m_sketches(cameras.size())
 , m_options(options)
 , m_running(false)
{
    for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
    {
        m_images.at(i) = new AtomicData<cv::Mat>();
        m_camOdoCompleted[i] = false;

        CamOdoThread* thread = new CamOdoThread(options.poseSource, options.nMotions, i, options.preprocessImages,
                                                m_images.at(i), m_cameras.at(i),
                                                m_odometryBuffer, m_interpOdometryBuffer, m_odometryBufferMutex,
                                                m_gpsInsBuffer, m_interpGpsInsBuffer, m_gpsInsBufferMutex,
                                                m_sketches.at(i), m_camOdoCompleted[i], m_stop,
                                                options.minKeyframeDistance, options.minVOSegmentSize,
                                                options.verbose);
        m_camOdoThreads.at(i) = thread;
        thread->signalFinished().connect(boost::bind(&CamRigOdoCalibration::onCamOdoThreadFinished, this, thread));
    }

    m_camOdoWatchdogThread = new CamOdoWatchdogThread(m_camOdoCompleted, m_stop);

    for (size_t i = 0; i < m_sketches.size(); ++i)
    {
        m_sketches.at(i) = cv::Mat(cameras.at(i)->imageHeight(), cameras.at(i)->imageWidth(), CV_8UC3);
        m_sketches.at(i) = cv::Scalar(0);
    }
}

CamRigOdoCalibration::~CamRigOdoCalibration()
{

}

// 设置初始的相机里程计位姿估计
void CamRigOdoCalibration::setInitialCameraOdoTransformEstimates(unsigned camIdx, const Eigen::Matrix4d& odoT)
{
    // 相机数量与相机里程计？？线程维度不一致，则返回
    if (camIdx >= m_camOdoThreads.size()) return;
    // 与相机数量对应的线程关闭或正在运行，则返回
    if (!m_camOdoThreads[camIdx] || m_camOdoThreads[camIdx]->running()) return;

    // 为该相机里程计线程设置相机里程计外参初始估计
    m_camOdoThreads[camIdx]->setCamOdoTransformEstimate(odoT);
}

// 添加图像
/**
 * @param cameraId   第几个相机
 * @param image      根据图像地址读取的图像
 * @param timestamp  时间戳
 */
void CamRigOdoCalibration::addFrame(int cameraId, const cv::Mat& image,
                               uint64_t timestamp)
{
    AtomicData<cv::Mat>* frame = m_images.at(cameraId);

    frame->lockData(); // 加锁---------------

    image.copyTo(frame->data()); // 将image图像复制给frame的data

    frame->timeStamp() = timestamp; // 将image的时间戳复制给frame的时间戳

    frame->unlockData(); //解锁------------

    if (m_options.mode == OFFLINE)
    {
        frame->waitForProcessingDone();  // ?
    }
}

void
CamRigOdoCalibration::addFrameSet(const std::vector<cv::Mat>& images,
                                  uint64_t timestamp)
{
    if (images.size() != m_cameras.size())
    {
        std::cout << "# WARNING: Number of images does not match number of cameras." << std::endl;
        return;
    }

    std::vector<boost::shared_ptr<boost::thread> > threads(m_cameras.size());
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        threads.at(i) = boost::make_shared<boost::thread>(boost::bind(&CamRigOdoCalibration::addFrame, this,
                                                          i, images.at(i), timestamp));
    }

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        threads.at(i)->join();
    }
}

// 添加odometry信息
void CamRigOdoCalibration::addOdometry(double x, double y, double z,
                                  double yaw,
                                  uint64_t timestamp)
{
    OdometryPtr odometry = boost::make_shared<Odometry>();
    odometry->x() = x;
    odometry->y() = y;
    odometry->z() = z;
    odometry->yaw() = yaw;
    odometry->timeStamp() = timestamp;

    m_odometryBuffer.push(timestamp, odometry);
}

void
CamRigOdoCalibration::addGpsIns(double lat, double lon, double alt,
                                double qx, double qy, double qz, double qw,
                                uint64_t timestamp)
{
    // convert latitude/longitude to UTM coordinates
    // 将纬度/经度转换为UTM坐标
     double utmX, utmY;
     std::string utmZone;
     LLtoUTM(lat, lon, utmX, utmY, utmZone);

     PosePtr pose = boost::make_shared<Pose>();
     pose->rotation() = Eigen::Quaterniond(qw,qx,qy,qz);
     pose->translation() = Eigen::Vector3d(utmX, utmY, -alt);

     pose->timeStamp() = timestamp;

     m_gpsInsBuffer.push(timestamp, pose);
}

void
CamRigOdoCalibration::addGpsIns(double lat, double lon, double alt,
                                double roll, double pitch, double yaw,
                                uint64_t timestamp)
{
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                       * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    addGpsIns(lat, lon, alt, q.x(), q.y(), q.z(), q.w(), timestamp);
}

// Rig:rigid 刚体
// 相机刚体里程计标定
void CamRigOdoCalibration::start(void)
{
    if (m_options.beginStage == 0)
    {
        // boost::asio提供了一个跨平台的异步编程IO模型库
        // io_service类在多线程编程模型中提供了任务队列和任务分发功能
        boost::asio::io_service io;
        //定义一个100毫秒的计时器 ，这里指定的是绝对时间
        boost::asio::deadline_timer timer(io, boost::posix_time::milliseconds(100));
        bool closeWindow = false;

        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            // 新建一个显示窗口，
            cv::namedWindow(m_cameras.at(i)->cameraName());
        }

        // timer.async_wait(handler); 计时时间一到，开始执行handler函数
        timer.async_wait(boost::bind(&CamRigOdoCalibration::displayHandler, this, &timer, &closeWindow));

        boost::thread windowThread(boost::bind(&CamRigOdoCalibration::pollWindow, this, &io, &closeWindow));

        std::cout << "# INFO: Running camera-odometry calibration for each of the " << m_cameras.size() << " cameras." << std::endl;

        // run odometry-camera calibration for each camera
        for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
        {
            m_camOdoThreads.at(i)->launch();
        }

        m_camOdoWatchdogThread->launch();
        m_camOdoWatchdogThread->join();

        for (size_t i = 0; i < m_camOdoThreads.size(); ++i)
        {
            m_camOdoThreads.at(i)->join();
        }

        closeWindow = true;
        windowThread.join();

        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            cv::destroyWindow(m_cameras.at(i)->cameraName());
        }

        buildGraph();

        m_running = true;

        std::cout << "# INFO: Completed camera-odometry calibration for all cameras." << std::endl;

        if (m_options.saveWorkingData)
        {
            if (!boost::filesystem::exists(m_options.dataDir))
            {
                boost::filesystem::create_directory(m_options.dataDir);
            }

            // save intermediate data
            std::cout << "# INFO: Saving intermediate data... " << std::flush;

            double tsStart = timeInSeconds();

            boost::filesystem::path extrinsicPath(m_options.dataDir);
            extrinsicPath /= "extrinsic_0";

            m_cameraSystem.writeToDirectory(extrinsicPath.string());

            boost::filesystem::path graphPath(m_options.dataDir);
            graphPath /= "frames_0.sg";
            m_graph.writeToBinaryFile(graphPath.string());
            // INFO: Saving intermediate data... Done. Took 1.72s
            std::cout << "Done. Took " << std::fixed << std::setprecision(2) << timeInSeconds() - tsStart << "s." << std::endl;
        }
    }

    std::cout << "# INFO: Running camera rig calibration." << std::endl;

    double tsStart = timeInSeconds(); // 获取系统当前时间

    // run calibration steps
    // m_graph.frameSetSegments().size() : 1
    // m_options.windowDistance : 3
    CameraRigBA ba(m_cameraSystem, m_graph, m_options.windowDistance);
    ba.setVerbose(m_options.verbose);

    // m_options.saveWorkingData : 1
    // m_options.dataDir : /data
    ba.run(m_options.beginStage, m_options.optimizeIntrinsics, m_options.saveWorkingData, m_options.dataDir);

    std::cout << "# INFO: Camera rig calibration took " << timeInSeconds() - tsStart << "s." << std::endl;
    std::cout << "# INFO: Completed camera rig calibration." << std::endl;

    m_running = false;
}

void
CamRigOdoCalibration::run(void)
{
    m_stop = true;
}

bool
CamRigOdoCalibration::isRunning(void) const
{
    return m_running;
}

const CameraSystem&
CamRigOdoCalibration::cameraSystem(void) const
{
    return m_cameraSystem;
}

void
CamRigOdoCalibration::onCamOdoThreadFinished(CamOdoThread* camOdoThread)
{
    if (m_options.verbose)
    {
        double minError, maxError, avgError;
        camOdoThread->reprojectionError(minError, maxError, avgError);

        std::cout << "# INFO: Reprojection error for camera " << camOdoThread->cameraId()
                  << ": avg = " << avgError
                  << " px | max = " << maxError << " px" << std::endl;
    }
}

bool compareFrameTimeStamp(FramePtr f1, FramePtr f2)
{
    return (f1->cameraPose()->timeStamp() < f2->cameraPose()->timeStamp());
}

// 建立spase maps ，对于论文每个相机会产生m个spase maps；
void CamRigOdoCalibration::buildGraph(void)
{
    boost::icl::interval_map<uint64_t, std::set<int> > intervals;  // 每个frameSegments时间范围对应一个相机集合
    // m_camOdoThreads.size() : 1
    std::vector<std::set<int> > cameraIdSets(m_camOdoThreads.size());

    for (size_t i = 0; i < m_camOdoThreads.size(); ++i) // 对于每一个camodo线程
    {
        CamOdoThread* camOdoThread = m_camOdoThreads.at(i);

        // 将m_camOdoThread的相机ID和m_cameras的相机id对应起来 // camOdoThread用于计算cam-odo-transform // m_cameras 存储相机模型
        m_cameraSystem.setCamera(camOdoThread->cameraId(), m_cameras.at(camOdoThread->cameraId()));
        // 将m_camOdoThread的相机ID和m_globalPoses相机id对应的位姿对应起来
        m_cameraSystem.setGlobalCameraPose(camOdoThread->cameraId(),
                                           camOdoThread->camOdoTransform());

        cameraIdSets[i].insert(camOdoThread->cameraId());  // 存放相机的编号 （相机编号set存相机编号？）

        for (size_t j = 0; j < camOdoThread->frameSegments().size(); ++j)
        {
            // 开始时间是camodo线程第j个frameSegments 的起始相机位姿的时间戳
            uint64_t start = camOdoThread->frameSegments().at(j).front()->cameraPose()->timeStamp();
            // 开始时间是camodo线程第j个frameSegments 的最后一个相机位姿的时间戳
            uint64_t end = camOdoThread->frameSegments().at(j).back()->cameraPose()->timeStamp();

            // 时间范围对应一个相机ID
            intervals += std::make_pair(boost::icl::interval<uint64_t>::right_open(start, end),
                                        cameraIdSets[i]);
        }
    }

    uint64_t lastIntervalEnd = 0;
    size_t intervalCounter = 0;

    boost::icl::interval_map<uint64_t, std::set<int> >::iterator it = intervals.begin();
    while (it != intervals.end())
    {
        // 时间间隔
        boost::icl::interval<uint64_t>::type interval = it->first;
        // 相机id  // cameraIds : 0
        std::set<int> cameraIds = it->second;

        //start : 1620458322569145
        //end : 1620458407405891
        uint64_t start = interval.lower(); // 图片起始时间戳
        uint64_t end = interval.upper();

        if (start != lastIntervalEnd)
        {
            // frameSetSegments 增加一个
            m_graph.frameSetSegments().resize(m_graph.frameSetSegments().size() + 1);
        }

        std::vector<FramePtr> frames;     //帧指针的集合

        std::set<int>::iterator itCameraId = cameraIds.begin();

        // 遍历每个相机
        while (itCameraId != cameraIds.end())
        {
            int cameraId = *itCameraId;

            // 每个cam-odom-thread
            CamOdoThread* camOdoThread = m_camOdoThreads.at(cameraId);

            // 对于每个frameSegments
            for (size_t j = 0; j < camOdoThread->frameSegments().size(); ++j)
            {
                // 对于每个frameSegment
                const std::vector<FramePtr>& frameSegment = camOdoThread->frameSegments().at(j);

                for (size_t k = 0; k < frameSegment.size(); ++k)
                {
                    // 对于每个frame
                    const FramePtr& frame = frameSegment.at(k);

                    // 每一帧的时间戳
                    uint64_t timestamp = frame->cameraPose()->timeStamp();
                    if (lastIntervalEnd == 0) //如果是第一帧
                    {
                        if (timestamp < start || timestamp > end)
                        {
                            continue;
                        }
                    }
                    else //不是第一帧
                    {
                        if (timestamp <= start || timestamp > end)
                        {
                            continue;
                        }
                    }

                    // 时间戳满足在时间范围内的frame 存入frames中
                    frames.push_back(frame);
                }
            }

            ++itCameraId;
        }

        // sort frames by timestamp
        std::sort(frames.begin(), frames.end(), compareFrameTimeStamp);

        size_t frameId = 0;
        while (frameId < frames.size())
        {
            FrameSetPtr frameSet = boost::make_shared<FrameSet>();               // FrameSet的指针
            frameSet->frames().resize(m_cameras.size());                // 大小 = 相机数目
            frameSet->systemPose() = frames.at(frameId)->systemPose();           // 该帧的里程计位姿
            frameSet->odometryMeasurement() = frames.at(frameId)->odometryMeasurement(); // 该帧的里程计位姿测量值
            frameSet->gpsInsMeasurement() = frames.at(frameId)->gpsInsMeasurement();     // 该帧的GPS位姿测量值

            uint64_t timestamp = frames.at(frameId)->cameraPose()->timeStamp();  // 该帧的时间戳
            while (frameId < frames.size() &&
                   frames.at(frameId)->cameraPose()->timeStamp() == timestamp)
            {
                frameSet->frames().at(frames.at(frameId)->cameraId()) = frames.at(frameId);

                ++frameId;
            }

            for (size_t i = 0; i < frameSet->frames().size(); ++i)
            {
                if (frameSet->frames().at(i).get() != 0)
                {
                    // frameSet中每一帧的位姿数据 与 frameSet的位姿数据一致
                    frameSet->frames().at(i)->systemPose() = frameSet->systemPose();
                    frameSet->frames().at(i)->odometryMeasurement() = frameSet->odometryMeasurement();
                    frameSet->frames().at(i)->gpsInsMeasurement() = frameSet->gpsInsMeasurement();
                }
            }

            m_graph.frameSetSegments().back().push_back(frameSet);
        }

        // 最新的interval的最后一帧 = end
        lastIntervalEnd = end;

        ++intervalCounter;
        ++it;
    }
}

void
CamRigOdoCalibration::pollWindow(boost::asio::io_service* io, bool* stop)
{
    while (!(*stop))
    {
        io->poll();

        usleep(1000);
    }
}

void CamRigOdoCalibration::displayHandler(boost::asio::deadline_timer* timer, bool* stop)
{
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        cv::imshow(m_cameras.at(i)->cameraName(), m_sketches.at(i));
    }
    cv::waitKey(2);

    if (!(*stop))
    {
        // expires_at 延时函数
        timer->expires_at(timer->expires_at() + boost::posix_time::milliseconds(100));
        // 计时时间一到，开始执行handler函数
        timer->async_wait(boost::bind(&CamRigOdoCalibration::displayHandler, this, timer, stop));
    }
}

}
