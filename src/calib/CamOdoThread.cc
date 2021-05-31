#include "CamOdoThread.h"

#include <boost/make_shared.hpp>
#include <iostream>

#ifdef HAVE_OPENCV3
#include <opencv2/imgproc.hpp>
#endif

#include "../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#include "../visual_odometry/FeatureTracker.h"
#include "utils.h"

#ifdef VCHARGE_VIZ
#include "../../../../library/gpl/CameraEnums.h"
#endif

namespace camodocal
{
//cam-odo-transform 计算线程
CamOdoThread::CamOdoThread(PoseSource poseSource,   // PoseSource::GPS_INS : PoseSource::ODOMETRY; 位姿来源
                           int nMotions,    // 关键帧数目
                           int cameraId,    // 第几个相机
                           bool preprocess, // 是否（有？）预处理图片
                           AtomicData<cv::Mat>* image,
                           const CameraConstPtr& camera,   // 相机指针
                           SensorDataBuffer<OdometryPtr>& odometryBuffer,  //里程计数据缓存器
                           SensorDataBuffer<OdometryPtr>& interpOdometryBuffer,
                           boost::mutex& odometryBufferMutex,
                           SensorDataBuffer<PosePtr>& gpsInsBuffer,
                           SensorDataBuffer<PosePtr>& interpGpsInsBuffer,
                           boost::mutex& gpsInsBufferMutex,
                           cv::Mat& sketch,
                           bool& completed,
                           bool& stop,
                           double minKeyframeDistance,     // 最小关键帧距离
                           size_t minVOSegmentSize,        // 最小VO segmen大小
                           bool verbose)
 : m_poseSource(poseSource)     // PoseSource::GPS_INS : PoseSource::ODOMETRY; 位姿来源
 , m_cameraId(cameraId)         // 第几个相机
 , m_preprocess(preprocess)     // 是否（有？）预处理图片
 , m_running(false)
 , m_image(image)
 , m_camera(camera)             // 相机指针
 , m_odometryBuffer(odometryBuffer)
 , m_interpOdometryBuffer(interpOdometryBuffer)
 , m_odometryBufferMutex(odometryBufferMutex)
 , m_gpsInsBuffer(gpsInsBuffer)
 , m_interpGpsInsBuffer(interpGpsInsBuffer)
 , m_gpsInsBufferMutex(gpsInsBufferMutex)
 , m_camOdoTransform(Eigen::Matrix4d::Identity())
 , m_camOdoTransformUseEstimate(false)
 , m_sketch(sketch)
 , m_completed(completed)
 , m_stop(stop)
 , k_minKeyframeDistance(minKeyframeDistance)   // 最小关键帧距离
 , k_minVOSegmentSize(minVOSegmentSize)         // 最小VO segmen大小
 , k_odometryTimeout(4.0)
{
    m_camOdoCalib.setVerbose(verbose);
    m_camOdoCalib.setMotionCount(nMotions);
}

CamOdoThread::~CamOdoThread()
{

}

int
CamOdoThread::cameraId(void) const
{
    return m_cameraId;
}

void
CamOdoThread::setCamOdoTransformEstimate(const Eigen::Matrix4d& estimate)
{
    m_camOdoTransform = estimate;
    m_camOdoTransformUseEstimate = true;
}


const Eigen::Matrix4d&
CamOdoThread::camOdoTransform(void) const
{
    return m_camOdoTransform;
}

const std::vector<std::vector<FramePtr> >& CamOdoThread::frameSegments(void) const
{
    return m_frameSegments;
}

void
CamOdoThread::reprojectionError(double& minError, double& maxError, double& avgError) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    for (size_t segmentId = 0; segmentId < m_frameSegments.size(); ++segmentId)
    {
        const std::vector<FramePtr>& segment = m_frameSegments.at(segmentId);

        for (size_t frameId = 0; frameId < segment.size(); ++frameId)
        {
            const FrameConstPtr& frame = segment.at(frameId);

            const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

            for (size_t i = 0; i < features2D.size(); ++i)
            {
                const Point2DFeatureConstPtr& feature2D = features2D.at(i);
                const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

                if (feature3D.get() == 0)
                {
                    continue;
                }

                double error = m_camera->reprojectionError(feature3D->point(),
                                                           frame->cameraPose()->rotation(),
                                                           frame->cameraPose()->translation(),
                                                           Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));

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

void
CamOdoThread::launch(void)
{
    m_running = true;

    m_thread = boost::make_shared<boost::thread>(&CamOdoThread::threadFunction, this);
}

void
CamOdoThread::join(void)
{
    if (m_running)
    {
        m_thread->join();
    }
}

bool
CamOdoThread::running(void) const
{
    return m_running;
}

boost::signals2::signal<void ()>&
CamOdoThread::signalFinished(void)
{
    return m_signalFinished;
}


// TODO 进程
void CamOdoThread::threadFunction(void)
{
    // 生成临时跟踪器
    TemporalFeatureTracker tracker(m_camera,
                                   SURF_GPU_DETECTOR, SURF_GPU_DESCRIPTOR,
                                   RATIO_GPU, m_preprocess, m_camOdoTransform);
    tracker.setVerbose(m_camOdoCalib.getVerbose()); //Verbose:日志显示

    FramePtr framePrev;

    cv::Mat image;
    cv::Mat colorImage;

    int trackBreaks = 0;

    std::vector<OdometryPtr> odometryPoses; // 里程计位姿

#ifdef VCHARGE_VIZ
    std::ostringstream oss;
    oss << "swba" << m_cameraId + 1;
    vcharge::GLOverlayExtended overlay(oss.str(), VCharge::COORDINATE_FRAME_GLOBAL);
#endif

    bool halt = false;

    while (!halt)  // 不暂停
    {
        // boost::get_system_time():获取系统当前时间;boost::posix_time::microseconds() 换算为指定的时间段
        boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(10);
        while (!m_image->timedWaitForData(timeout) && !m_stop)
        {
            timeout = boost::get_system_time() + boost::posix_time::milliseconds(10);
        }

        if (m_stop) // 与跟踪失败有关
        {
            // 获得相机位姿
            std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > voPoses = tracker.getPoses();

            if (odometryPoses.size() >= k_minVOSegmentSize) // 如果里程计位姿的维度大于等于最小VO分割尺度
            {
                // 添加m_frameSegments 数据（相机，里程计位姿）
                addCamOdoCalibData(voPoses, odometryPoses, tracker.getFrames());
            }

            // 如果里程计位姿不为空，跟踪失败的位姿都删去
            if (!odometryPoses.empty())
            {
                odometryPoses.erase(odometryPoses.begin(), odometryPoses.begin() + voPoses.size() - 1);
            }

            ++trackBreaks; // 跟踪失败+1

            halt = true;
        }
        else // !m_stop
        {
            m_image->lockData(); // 加锁××××××××××××××××××××××××××××
            m_image->available() = false;

            // 图像的时间戳
            uint64_t timeStamp = m_image->timeStamp();

            // 如果图像上一帧的时间戳和当前帧相同
            if (framePrev.get() != 0 && timeStamp == framePrev->cameraPose()->timeStamp())
            {
                m_image->unlockData();
                m_image->notifyProcessingDone();

                continue;
            }

            m_image->data().copyTo(image);

            m_image->unlockData();  // 解锁××××××××××××××××××××××××××××

            if (image.channels() == 1)
            {
                cv::cvtColor(image, colorImage, CV_GRAY2BGR);
            }
            else
            {
                image.copyTo(colorImage);
            }

            // skip if current car position is too near previous position
            // 如果当前车辆位置太接近上一个位置，则跳过
            OdometryPtr currOdometry;
            Eigen::Vector2d pos;

            if (m_poseSource == ODOMETRY && !m_odometryBuffer.current(currOdometry))
            {
                std::cout << "# WARNING: No data in odometry buffer." << std::endl;
            }
            else
            {
                m_odometryBufferMutex.lock();

                OdometryPtr interpOdo;
                // 如果位姿的来源是odometry，且没找到图像时间戳对应的里程计信息
                if (m_poseSource == ODOMETRY && !m_interpOdometryBuffer.find(timeStamp, interpOdo))
                {
                    double timeStart = timeInSeconds();

                    // 插值里程计信息失败
                    while (!interpolateOdometry(m_odometryBuffer, timeStamp, interpOdo))
                    {
                        if (timeInSeconds() - timeStart > k_odometryTimeout)
                        {
                            std::cout << "# ERROR: No odometry data for " << k_odometryTimeout << "s. Exiting..." << std::endl;
                            exit(1);
                        }

                        usleep(1000);
                    }


                    m_interpOdometryBuffer.push(timeStamp, interpOdo);
                }

                m_odometryBufferMutex.unlock();


                Eigen::Vector3d pos;
                if (m_poseSource == ODOMETRY)
                {
                    pos = interpOdo->position();
                }

                if (framePrev.get() != 0 &&
                    (pos - framePrev->systemPose()->position()).norm() < k_minKeyframeDistance)
                {
                    m_image->notifyProcessingDone();
                    continue;
                }

                FramePtr frame = boost::make_shared<Frame>();
                frame->cameraId() = m_cameraId;
                image.copyTo(frame->image());

                bool camValid = tracker.addFrame(frame, m_camera->mask());

                if (interpOdo)
                {
                    frame->odometryMeasurement() = boost::make_shared<Odometry>();
                    *(frame->odometryMeasurement()) = *interpOdo;
                    frame->systemPose() = boost::make_shared<Odometry>();
                    *(frame->systemPose()) = *interpOdo;
                }

                frame->cameraPose()->timeStamp() = timeStamp;

                if (camValid)
                {
                    odometryPoses.push_back(frame->systemPose());
                }

                framePrev = frame;

                if (!camValid)
                {
                    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > voPoses = tracker.getPoses();

                    if (odometryPoses.size() >= k_minVOSegmentSize)
                    {
                        addCamOdoCalibData(voPoses, odometryPoses, tracker.getFrames());
                    }

                    if (!odometryPoses.empty())
                    {
                        odometryPoses.erase(odometryPoses.begin(), odometryPoses.begin() + voPoses.size() - 1);
                    }

                    ++trackBreaks;
                }
            }
        }

#ifdef VCHARGE_VIZ
        visualizeMap(overlay, tracker);
#endif

        int currentMotionCount = 0;
        if (odometryPoses.size() >= k_minVOSegmentSize)
        {
            currentMotionCount = odometryPoses.size() - 1;
        }

        // visualize feature tracks
        std::ostringstream oss;
        oss << "# motions: " << m_camOdoCalib.getCurrentMotionCount() + currentMotionCount << " | "
            << "# track breaks: " << trackBreaks;

        std::string status = oss.str();

        if (!tracker.getSketch().empty())
        {
            tracker.getSketch().copyTo(m_sketch);
        }
        else
        {
            colorImage.copyTo(m_sketch);
        }

        int fontFace = cv::FONT_HERSHEY_COMPLEX;
        double fontScale = 0.5;
        int thickness = 1;

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(status, fontFace,
                                            fontScale, thickness, &baseline);
        baseline += thickness;

        // center the text horizontally and at bottom of image
        cv::Point textOrg((m_sketch.cols - textSize.width) / 2,
                           m_sketch.rows - textSize.height - 10);
        cv::putText(m_sketch, status, textOrg, fontFace, fontScale,
                    cv::Scalar::all(255), thickness, CV_AA);

        m_image->notifyProcessingDone();

        if (m_camOdoCalib.getCurrentMotionCount() + currentMotionCount >= m_camOdoCalib.getMotionCount())
        {
            m_completed = true;
        }
    }

    if (!m_camOdoTransformUseEstimate)
    {

        Eigen::Matrix4d H_cam_odo;

        m_camOdoCalib.calibrate(H_cam_odo);
        std::cout <<"m_camOdoTransform = H_cam_odo : "  << H_cam_odo << std::endl;
        m_camOdoTransform = H_cam_odo;
    }

    {
        static boost::mutex mutex;
        boost::mutex::scoped_lock lock(mutex);

        if (m_camOdoTransformUseEstimate)
            std::cout << "# INFO: Use provided odometry estimate for camera " << m_cameraId << "..." << std::endl;
        else
            // # INFO: Calibrating odometry - camera 0...
            std::cout << "# INFO: Calibrating odometry - camera " << m_cameraId << "..." << std::endl;

        std::cout << "Rotation: " << std::endl << m_camOdoTransform.block<3,3>(0,0) << std::endl;
        std::cout << "Translation: " << std::endl << m_camOdoTransform.block<3,1>(0,3).transpose() << std::endl;

    }

    m_running = false;

    m_signalFinished();
}

// 添加m_frameSegments 数据（相机，里程计位姿）
void CamOdoThread::addCamOdoCalibData(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& camPoses,
                                 const std::vector<OdometryPtr>& odoPoses,
                                 std::vector<FramePtr>& frameSegment)
{
    if (odoPoses.size() != camPoses.size())
    {
        std::cout << "# WARNING: Numbers of odometry (" << odoPoses.size()
                  << ") and camera poses (" << camPoses.size() << ") differ. Aborting..." << std::endl;

        return;
    }

    if (odoPoses.size() < k_minVOSegmentSize)
    {
        std::cout << "# WARNING: At least " << k_minVOSegmentSize << " poses are needed. Aborting..." << std::endl;

        return;
    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > odoMotions;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > camMotions;


    for (size_t i = 1; i < odoPoses.size(); ++i)
    {
        /// 因为整个系统是刚体，所以相同变化下里程计的旋转和相机的旋转是一致的。认为这里可能是为了两个相乘求误差
        // To(i) * R = To(i-1)
        Eigen::Matrix4d relativeOdometryPose = odoPoses.at(i)->toMatrix().inverse() * odoPoses.at(i - 1)->toMatrix();
        odoMotions.push_back(relativeOdometryPose);

        // Tc(i) = R * Tc(i-1)
        Eigen::Matrix4d relativeCameraPose = camPoses.at(i) * camPoses.at(i - 1).inverse();
        camMotions.push_back(relativeCameraPose);

    }

    if (!m_camOdoCalib.addMotionSegment(camMotions, odoMotions))
    {
        std::cout << "# ERROR: Numbers of odometry and camera motions do not match." << std::endl;
        exit(0);
    }

    m_frameSegments.push_back(frameSegment);
}

#ifdef VCHARGE_VIZ

void
CamOdoThread::visualizeMap(vcharge::GLOverlayExtended& overlay,
                           TemporalFeatureTracker& tracker)
{
    // visualize camera poses and 3D scene points
    const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& poses = tracker.getPoses();

    overlay.clear();
    overlay.pointSize(2.0f);
    overlay.lineWidth(1.0f);

    // draw 3D scene points
//    switch (cameraId)
//    {
//    case CAMERA_FRONT:
//        overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
//        break;
//    case CAMERA_LEFT:
//        overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
//        break;
//    case CAMERA_REAR:
//        overlay.color4f(0.0f, 0.0f, 1.0f, 0.5f);
//        break;
//    case CAMERA_RIGHT:
//        overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
//        break;
//    default:
//        overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
//    }
//
//    overlay.begin(VCharge::POINTS);
//
//    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints = tracker.getScenePoints();
//    for (size_t j = 0; j < scenePoints.size(); ++j)
//    {
//        Eigen::Vector3d& p = scenePoints.at(j);
//
//        overlay.vertex3f(p(2), -p(0), -p(1));
//    }
//
//    overlay.end();

    // draw cameras
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Eigen::Matrix4d H = poses.at(i).inverse();

        double xBound = 0.1;
        double yBound = 0.1;
        double zFar = 0.2;

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > frustum;
        frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        frustum.push_back(Eigen::Vector3d(-xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, yBound, zFar));
        frustum.push_back(Eigen::Vector3d(-xBound, yBound, zFar));

        for (size_t j = 0; j < frustum.size(); ++j)
        {
            frustum.at(j) = transformPoint(H, frustum.at(j));
        }

        overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
        overlay.begin(VCharge::LINES);

        for (int j = 1; j < 5; ++j)
        {
            overlay.vertex3f(frustum.at(0)(2), -frustum.at(0)(0), -frustum.at(0)(1));
            overlay.vertex3f(frustum.at(j)(2), -frustum.at(j)(0), -frustum.at(j)(1));
        }

        overlay.end();

        switch (m_cameraId)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 0.0f, 1.0f, 0.5f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
        }

        overlay.begin(VCharge::POLYGON);

        for (int j = 1; j < 5; ++j)
        {
            overlay.vertex3f(frustum.at(j)(2), -frustum.at(j)(0), -frustum.at(j)(1));
        }

        overlay.end();
    }

    overlay.publish();
}

#endif

}
