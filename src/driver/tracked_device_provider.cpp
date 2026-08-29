#include "tracked_device_provider.h"

#include "interface_hook_injector.h"
#include "log.h"
#include "util.h"
#include "virtual_desktop.h"
#include "vrmath.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <openvr_driver.h>

namespace spacecal {
vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    util::init();
    logging::Init(/* isOverlay */ false);
    hmd::initVD();

    LOG_OPENVR_INFO("Starting SpaceCalibrator-Nova. Compiled against OpenVR driver API {}.{}.{}", vr::k_nSteamVRVersionMajor, vr::k_nSteamVRVersionMinor, vr::k_nSteamVRVersionBuild);

    m_alignmentParams.thr_rot_tiny = 0.1f * (EIGEN_PI / 180.0f);
    m_alignmentParams.thr_rot_small = 1.0f * (EIGEN_PI / 180.0f);
    m_alignmentParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

    m_alignmentParams.thr_trans_tiny = 0.1f / 1000.0; // mm
    m_alignmentParams.thr_trans_small = 1.0f / 1000.0; // mm
    m_alignmentParams.thr_trans_large = 20.0f / 1000.0; // mm

    m_alignmentParams.align_speed_tiny = 0.05f;
    m_alignmentParams.align_speed_small = 0.2f;
    m_alignmentParams.align_speed_large = 2.0f;

    m_ipcServer.Connect(this);
    hooking::InjectHooks(this, pDriverContext);

    return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
    LOG_OPENVR_INFO("Shutting down SpaceCalibrator-Nova...");

    hooking::DisableHooks();
    m_ipcServer.Shutdown();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* ServerTrackedDeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}
void ServerTrackedDeviceProvider::RunFrame()
{
    // we query hmd refresh rate for prediction
    vr::PropertyContainerHandle_t hHmdContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
    m_fVsyncPredictionTime = 1.0 / vr::VRProperties()->GetFloatProperty(hHmdContainer, vr::Prop_DisplayFrequency_Float);
    m_latencyEstimator.set_hmd_refresh_rate(m_fVsyncPredictionTime);

    // device state may get desynced so we update it here
    vr::TrackedDevicePose_t rawPoses[vr::k_unMaxTrackedDeviceCount] = {};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses((float)m_fVsyncPredictionTime, rawPoses, vr::k_unMaxTrackedDeviceCount);

    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        if (IsPoseValid(m_poses[i]) && !IsPoseValid(rawPoses[i])) {
            vr::DriverPose_t corrected = m_poses[i];
            corrected.deviceIsConnected = rawPoses[i].bDeviceIsConnected;
            corrected.poseIsValid = rawPoses[i].bPoseIsValid;
            corrected.result = rawPoses[i].eTrackingResult;
            m_ipcServer.UpdatePose(i, corrected);
        }
    }
}
bool ServerTrackedDeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

// @TODO: should spacecal inform the overlay to ignore calibrations during standby?
void ServerTrackedDeviceProvider::EnterStandby() { }
void ServerTrackedDeviceProvider::LeaveStandby() { }

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double (&vector)[3])
{
    vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1], vector[2] };
    auto rotatedVectorQuat = quat * vectorQuat * -quat;
    return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}
inline void copyVec3(double dest[3], const vr::HmdVector3d_t& src)
{
    ASSERT(dest != nullptr, "dest was not a valid memory address!");
    dest[0] = src.v[0];
    dest[1] = src.v[1];
    dest[2] = src.v[2];
}

inline vr::HmdQuaternion_t hmdQuatFromEigen(const Eigen::Quaterniond& quat)
{
    return vr::HmdQuaternion_t { quat.w(), quat.x(), quat.y(), quat.z() };
}
inline vr::HmdVector3d_t hmdVecFromEigenVec(const Eigen::Vector3d& pos)
{
    return vr::HmdVector3d_t { .v = { pos.x(), pos.y(), pos.z() } };
}
inline vr::HmdVector3d_t hmdVecFromEigenVec(const Eigen::Translation3d& pos)
{
    return vr::HmdVector3d_t { .v = { pos.x(), pos.y(), pos.z() } };
}

inline Eigen::Quaterniond eigenFromHmdQuat(const vr::HmdQuaternion_t& quat)
{
    return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}
inline Eigen::Translation3d eigenTransFromHmdVec(const double pos[3])
{
    return Eigen::Translation3d(pos[0], pos[1], pos[2]);
}
inline Eigen::Vector3d eigenVecFromHmdVec(const double pos[3])
{
    return Eigen::Vector3d(pos[0], pos[1], pos[2]);
}

inline DeltaSize maxDelta(DeltaSize a, DeltaSize b)
{
    return (static_cast<int>(a) > static_cast<int>(b)) ? a : b;
}

DeltaSize ServerTrackedDeviceProvider::getTransformDeltaSize(DeltaSize priorDelta, const Eigen::Vector3d& deviceWorldPos, const Pose_t& current, const Pose_t& target) const
{
    const Eigen::Vector3d srcPos = current.pos + current.rot * deviceWorldPos;
    const Eigen::Vector3d targetPos = target.pos + target.rot * deviceWorldPos;

    const double transDelta = (srcPos - targetPos).squaredNorm();
    const double rotDelta = current.rot.angularDistance(target.rot);

    DeltaSize transLevel, rotLevel;

    if (transDelta > m_alignmentParams.thr_trans_large)
        transLevel = DeltaSize::LARGE;
    else if (transDelta > m_alignmentParams.thr_trans_small)
        transLevel = DeltaSize::SMALL;
    else
        transLevel = DeltaSize::TINY;

    if (rotDelta > m_alignmentParams.thr_rot_large)
        rotLevel = DeltaSize::LARGE;
    else if (rotDelta > m_alignmentParams.thr_rot_small)
        rotLevel = DeltaSize::SMALL;
    else
        rotLevel = DeltaSize::TINY;

    if (transLevel == DeltaSize::TINY && rotLevel == DeltaSize::TINY)
        return DeltaSize::TINY;

    return maxDelta(priorDelta, maxDelta(transLevel, rotLevel));
}

double ServerTrackedDeviceProvider::getTransformRate(DeltaSize delta) const
{
    switch (delta) {
    case DeltaSize::TINY:
        return m_alignmentParams.align_speed_tiny;
    case DeltaSize::SMALL:
        return m_alignmentParams.align_speed_small;
    default:
        return m_alignmentParams.align_speed_large;
    }
}

// lerps between two calibrations by lerp factor % around local point
inline Pose_t interpolatePoseAround(double lerp, const Eigen::Vector3d& localPoint, const Pose_t& current, const Pose_t& target)
{
    const Eigen::Vector3d initialWorld = current.pos + current.rot * localPoint;
    const Eigen::Vector3d targetWorld = target.pos + target.rot * localPoint;
    const Eigen::Vector3d finalWorld = initialWorld * (1.0 - lerp) + targetWorld * lerp;

    Eigen::Quaterniond rot = current.rot.slerp(lerp, target.rot);
    Eigen::Vector3d pos = finalWorld - rot * localPoint;

    return Pose_t { .rot = rot, .pos = pos };
}

inline Eigen::Affine3d getWorldFromDriverPose(const vr::DriverPose_t& pose, double fPredictedSecondsToPhotonsFromNow = 0.0)
{
    Eigen::Quaterniond baseRot = eigenFromHmdQuat(pose.qRotation);
    Eigen::Vector3d basePos = eigenVecFromHmdVec(pose.vecPosition);

    // @NOTE: ignoring poseTimeOffset because hardware lies :(
    // double deltaTime = -pose.poseTimeOffset + fPredictedSecondsToPhotonsFromNow;
    double deltaTime = fPredictedSecondsToPhotonsFromNow;

    constexpr double k_PREDICTION_MIN_DELTA_TIME = 0.0001; // 0.1ms
    constexpr double k_PREDICTION_MIN_VELOCITY = 0.00001; // 0.00001 rad/s

    // we need to apply pose prediction here or the poses for relative calibration will be off.

    // kinematic displacement for constant acceleration
    // we assume constant accel and velocity as we have instantaneous vel and accel
    // taking an integral, as we approach the limit 0, we're effectively splitting the equation into multiple small units until they're infinitely small
    // suppose instead of taking the limit to 0 we take the limit to HMD_REFRESH_RATE instead, if we integrate using that assumption we're effectively
    // taking multiple instantaneous samples of the velocity and acceleration, so we can assume constant acceleration and velocity here for prediction purposes

    if (deltaTime > k_PREDICTION_MIN_DELTA_TIME) {
        // s = ut + (1/2)at^2
        Eigen::Vector3d vel = eigenVecFromHmdVec(pose.vecVelocity);
        Eigen::Vector3d accel = eigenVecFromHmdVec(pose.vecAcceleration);
        basePos += (vel * deltaTime) + (0.5 * accel * deltaTime * deltaTime);

        // angular prediction (rotation)
        // NOTE we IGNORE angular accel because its too noisy to be useful for prediction!
        Eigen::Vector3d angVel = eigenVecFromHmdVec(pose.vecAngularVelocity);
        double angVelMag = angVel.norm();
        if (angVelMag > k_PREDICTION_MIN_VELOCITY) {
            Eigen::Vector3d axis = angVel / angVelMag;
            double angle = angVelMag * deltaTime;
            Eigen::Quaterniond deltaRot(Eigen::AngleAxisd(angle, axis));
            baseRot = deltaRot * baseRot;
        }
    }

    Eigen::Quaterniond worldFromDriverRot = eigenFromHmdQuat(pose.qWorldFromDriverRotation);
    Eigen::Vector3d worldFromDriverPos = eigenVecFromHmdVec(pose.vecWorldFromDriverTranslation);

    Eigen::Quaterniond rot = worldFromDriverRot * baseRot;
    Eigen::Vector3d pos = worldFromDriverPos + worldFromDriverRot * basePos;

    return Eigen::Translation3d(pos) * rot;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(vr::TrackedDeviceIndex_t unWhichDevice, vr::DriverPose_t& newPose)
{

    // bounds check, as sometimes the id is invalid?
    if (!IsDeviceIndexValid(unWhichDevice))
        return true;

    m_ipcServer.UpdatePose(unWhichDevice, newPose);
    m_latencyEstimator.push_pose(unWhichDevice, newPose);

    vr::DriverPose_t& modifiedPose = newPose;

    auto& transform = m_transforms[unWhichDevice];

    if (transform.hideContinuousTracker()) {
        newPose.vecPosition[0] = -newPose.vecWorldFromDriverTranslation[0];
        newPose.vecPosition[1] = -newPose.vecWorldFromDriverTranslation[1] + 9001; // put it 9001m above the origin
        newPose.vecPosition[2] = -newPose.vecWorldFromDriverTranslation[2];
    } else if (transform.enabled()) {
        HandleQuirks(transform.quirks, modifiedPose);

        if (transform.relativeCoordSystem()) {
            if (IsDeviceIndexValid(transform.unRelativeReferenceOpenvrDeviceId) && IsPoseValid(m_poses[transform.unRelativeReferenceOpenvrDeviceId]) && IsDeviceIndexValid(transform.unRelativeTargetOpenvrDeviceId) && IsPoseValid(m_poses[transform.unRelativeTargetOpenvrDeviceId])) {
                const vr::DriverPose_t& refPose = m_poses[transform.unRelativeReferenceOpenvrDeviceId];
                const vr::DriverPose_t& targetPose = m_poses[transform.unRelativeTargetOpenvrDeviceId];

                // world-space pose of ref device (H')
                Eigen::Affine3d refWorldNow = getWorldFromDriverPose(refPose, m_fVsyncPredictionTime);
                // world-space pose of target device (T')
                double latency = m_latencyEstimator.get_latency(transform.unRelativeReferenceOpenvrDeviceId, transform.unRelativeTargetOpenvrDeviceId);
                Eigen::Affine3d targetWorldNow = getWorldFromDriverPose(targetPose, m_fVsyncPredictionTime + latency);

                // local-space calibration transform (C_L)
                Eigen::Affine3d calibrationLocal = eigenTransFromHmdVec(transform.translation.v) * eigenFromHmdQuat(transform.rotation);

                // To compute the calibration for now, we need to compute where we expect the tracker to be (by applying the local calibration),
                // and then computing a mapping from the tracker's raw pose to the expected pose to get a calibration thats valid for now.

                // T_H = H' * C_L
                Eigen::Affine3d expectedTargetWorld = refWorldNow * calibrationLocal;

                // C' = (T')^-1 * T_H
                Eigen::Affine3d worldCalib = expectedTargetWorld * targetWorldNow.inverse();

                // keep track of calibrations so that we can keep using them even when either the ref or target device lose tracking
                // we write directly to blend target for free smoothing
                auto now = std::chrono::steady_clock::now();

                Pose_t targetCalib = {
                    .rot = Eigen::Quaterniond(worldCalib.rotation()).normalized(),
                    .pos = worldCalib.translation(),
                };

                if (!m_cachedCalibrations[unWhichDevice].hasCalibration) {
                    m_cachedCalibrations[unWhichDevice] = {
                        .pose = targetCalib,
                        .eDeltaSize = DeltaSize::TINY,
                        .hasCalibration = true,
                        .lastUpdateTime = now,
                    };
                } else {
                    double deltaTime = std::chrono::duration<double>(now - m_cachedCalibrations[unWhichDevice].lastUpdateTime).count();
                    m_cachedCalibrations[unWhichDevice].lastUpdateTime = now;

                    Eigen::Vector3d deviceWorldPos = targetWorldNow.translation();

                    m_cachedCalibrations[unWhichDevice].eDeltaSize = getTransformDeltaSize(
                        m_cachedCalibrations[unWhichDevice].eDeltaSize,
                        deviceWorldPos,
                        m_cachedCalibrations[unWhichDevice].pose,
                        targetCalib);

                    double lerp = deltaTime * getTransformRate(m_cachedCalibrations[unWhichDevice].eDeltaSize);
                    lerp = std::clamp(lerp, 0.0, 1.0);

                    m_cachedCalibrations[unWhichDevice].pose = interpolatePoseAround(lerp, deviceWorldPos, m_cachedCalibrations[unWhichDevice].pose, targetCalib);
                }
            }

            applyCalibrationToPose(
                modifiedPose,
                hmdQuatFromEigen(m_cachedCalibrations[unWhichDevice].pose.rot),
                hmdVecFromEigenVec(m_cachedCalibrations[unWhichDevice].pose.pos),
                transform.scale,
                transform.calibrateMotionVecs());
        } else {
            // classic world-space application
            applyCalibrationToPose(modifiedPose, transform.rotation, transform.translation, transform.scale, transform.calibrateMotionVecs());
        }
    }

    return true;
}

void ServerTrackedDeviceProvider::applyCalibrationToPose(vr::DriverPose_t& pose, vr::HmdQuaternion_t rotation, vr::HmdVector3d_t pos, double scale, bool calibrateMotionVecs)
{
    // apply like classic world-space calibration
    pose.qWorldFromDriverRotation = rotation * pose.qWorldFromDriverRotation;

    pose.vecPosition[0] *= scale;
    pose.vecPosition[1] *= scale;
    pose.vecPosition[2] *= scale;

    vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(rotation, pose.vecWorldFromDriverTranslation);
    pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + pos.v[0];
    pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + pos.v[1];
    pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + pos.v[2];

    // @NOTE: remove branch in future update once i determine this works well
    if (calibrateMotionVecs) {
        // velocity
        pose.vecVelocity[0] *= scale;
        pose.vecVelocity[1] *= scale;
        pose.vecVelocity[2] *= scale;

        // acceleration
        pose.vecAcceleration[0] *= scale;
        pose.vecAcceleration[1] *= scale;
        pose.vecAcceleration[2] *= scale;
    }
}

void ServerTrackedDeviceProvider::HandleQuirks(ipc::protocol::DeviceQuirks_t quirks, vr::DriverPose_t& pose)
{
    // @TODO: implement quirks handling

    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_SCALE)) {
        // m_poses->vecAngularVelocity
    }

    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_RECOMPUTE_VELOCITY)) {
        // m_poses->vecAngularVelocity
    }

    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_RECOMPUTE_ANGULAR_VELOCITY)) {
        // m_poses->vecAngularVelocity
    }

    // this improves pose prediction on some vendors' devices (eg Pico)
    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_HAS_WORLDSPACE_ANGULAR_VELOCITY)) {
        // convert quat from world space to local space
        vr::HmdQuaternion_t qInverse = -pose.qRotation; // invert quaternion, i should probably make this an explicit function bc i dont like operators being used like this
        vr::HmdVector3d_t localAngularVel = quaternionRotateVector(qInverse, pose.vecAngularVelocity);
        copyVec3(pose.vecAngularVelocity, localAngularVel);
    }
}

void ServerTrackedDeviceProvider::ResetCalibration()
{
    // @TODO: idk what would be more correct than "random fucking offsets"
}

void ServerTrackedDeviceProvider::RequestVirtualDesktopProps()
{
    // this simply states whether or not the vd driver is loaded. it doesnt tell us if the active hmd IS vd
    m_hmdMetaData.isVirtualDesktopAvailable = vr::VRDriverManager()->GetDriverHandle("virtualdesktop") != vr::k_ulInvalidDriverHandle;
    // NOTE: DO NOT INVOKE IF VD IS NOT THE ACTIVE HMD! WILL CAUSE CRASH!!
    if (m_hmdMetaData.isVirtualDesktopAvailable) {
        m_hmdMetaData.VD_hmdModel = hmd::VD_getHmdModel();
        m_hmdMetaData.VD_stageTrackingEnabled = hmd::VD_isStageTrackingEnabled();
    }
}

void ServerTrackedDeviceProvider::SetAlignmentSpeedParams(ipc::protocol::Command_SetAlignmentSpeedParams_t& params)
{
    m_alignmentParams = params;
}
}