#include "utils.h"

#include "../gpl/gpl.h"

namespace camodocal
{


/**
 * 插入（插值）里程计信息
 * @param odometryBuffer 里程计信息集合
 * @param timestamp      图像时间戳
 * @param interpOdo
 * @return
 */
bool interpolateOdometry(SensorDataBuffer<OdometryPtr>& odometryBuffer,
                    uint64_t timestamp, OdometryPtr& interpOdo)
{
    OdometryPtr prev, next;

    if (!odometryBuffer.nearest(timestamp, prev, next))
    {
        return false;
    }

    interpOdo = OdometryPtr(new Odometry);
    interpOdo->timeStamp() = timestamp;

    // 如果里程计在目标时间戳前后帧位姿相同，则插值的里程计位姿和他们一样
    if (prev->position() == next->position())
    {
        interpOdo->position() = prev->position();
        interpOdo->attitude() = prev->attitude();

        return true;
    }

    double t = static_cast<double>(timestamp - prev->timeStamp()) / (next->timeStamp() - prev->timeStamp());

    interpOdo->position() = t * (next->position() - prev->position()) + prev->position();

    prev->yaw() = normalizeTheta(prev->yaw());
    next->yaw() = normalizeTheta(next->yaw());

    interpOdo->yaw() = t * normalizeTheta(next->yaw() - prev->yaw()) + prev->yaw();

    prev->pitch() = normalizeTheta(prev->pitch());
    next->pitch() = normalizeTheta(next->pitch());

    interpOdo->pitch() = t * normalizeTheta(next->pitch() - prev->pitch()) + prev->pitch();

    prev->roll() = normalizeTheta(prev->roll());
    next->roll() = normalizeTheta(next->roll());

    interpOdo->roll() = t * normalizeTheta(next->roll() - prev->roll()) + prev->roll();

    return true;
}

bool
interpolatePose(SensorDataBuffer<PosePtr>& poseBuffer,
                uint64_t timestamp, PosePtr& interpPose)
{
    PosePtr prev, next;

    if (!poseBuffer.nearest(timestamp, prev, next))
    {
        return false;
    }

    interpPose = PosePtr(new Pose);
    interpPose->timeStamp() = timestamp;

    if (prev->translation() == next->translation())
    {
        interpPose->rotation() = prev->rotation();
        interpPose->translation() = prev->translation();

        return true;
    }

    double t = static_cast<double>(timestamp - prev->timeStamp()) / (next->timeStamp() - prev->timeStamp());

    interpPose->rotation() = prev->rotation().slerp(t, next->rotation());
    interpPose->rotation().normalize();

    for (int i = 0; i < 3; ++i)
    {
        interpPose->translation()(i) = t * (next->translation()(i) - prev->translation()(i)) + prev->translation()(i);
    }

    return true;
}

}

