#pragma once

#include "ipc_client.h"
#include "openvr.h"
#include "protocol.h"
#include "time_series.h"
#include "vr_core.h"
#include <Eigen/Dense>
#include <deque>

namespace spacecal {

constexpr double k_TICK_RATE_HZ = 20.0; // tick rate spacecal's internal logic runs at
constexpr double k_MAX_INVALID_CALIBRATION_TIME_SEC = 60.0; // 60s between invalid calibrations
constexpr double k_IPC_CONNECTION_RETRY_INTERVAL_SEC = 0.25; // 250ms between IPC connection attempts
constexpr double k_AUTO_DETECT_DURATION = 5.0; // 2 seconds for auto-detection
constexpr int k_AUTO_DETECT_MINIMUM_SCORE = 40;
constexpr float k_AUTO_DETECT_MIN_VELOCITY_THRESHOLD = 0.1f; // devices need to be moving enough for us to consider them
constexpr float k_AUTO_DETECT_MAX_VELOCITY_DIFF = 0.15f; // speed tolerance between devices, we only allow for up to this much variance in speed due to hardware differences and unit to unit variance
constexpr double k_PLAYSPACE_JUMP_WORLD_FROM_DRIVER_ANGLE_THRESHOLD_RAD = 1.0 * (EIGEN_PI / 180.0); // the angular threshold of movement for detecting sudden playspace jumps from a device
constexpr double k_PLAYSPACE_JUMP_WORLD_FROM_DRIVER_POS_THRESHOLD_METERS = 0.005; // the threshold in metres for detecting sudden playspace jumps from a device

// thresholds and bounds to ensure the underlying mathematical algorithms maintain a high enough accuracy to be useful for real-world use.
constexpr double k_MAX_RETARGETING_RMS_ERROR_THRESHOLD = 0.1;
constexpr double k_MAX_AXIS_VARIANCE_THRESHOLD = 0.001;
constexpr double k_ROTATION_ANGLE_THRESHOLD = 0.4;
constexpr double k_ROTATION_MAGNITUDE_THRESHOLD = 0.1;
constexpr double k_ROTATION_MIN_VARIANCE = 0.15; // @TODO: fine tune
constexpr double k_TRANSLATION_MIN_MOTION_THRESHOLD = 0.01; // @TODO: fine tune
constexpr double k_TRANSLATION_MAX_AMPLIFIED_NOISE_FACTOR = 2.5; // @TODO: fine tune
constexpr size_t k_MIN_DELTA_SAMPLE_COUNT = 200;
constexpr double k_ROTATION_VARIANCE_TARGET_DEGREES = 14.0; // how much rotation variance to enforce in degrees

constexpr double k_METRIC_HISTORY_TIMESPAN = 60.0; // how much time in seconds we keep track of for the graphs
constexpr double k_WARN_DEVICE_NOT_TRACKING_INTERVAL_SEC = 5.0; // time in seconds between "Device isnt tracking" log msgs

enum class CalibrationState : uint8_t {
    // calibration is inactive
    NONE,
    // calibration is starting
    START,
    // collecting samples
    SAMPLE,
    // user is editing calibration
    EDITING,
    // continuous calibration mode
    CONTINUOUS,
    // the calibration was set to continuous in a previous session, but the user hasn't selected it. we won't compute for now
    CONTINUOUS_IDLE,
    // currently auto-detecting devices -> returns to standard calibration once done
    AUTO_DETECT_DEVICES_STANDARD,
    // currently auto-detecting devices -> returns to continuous calibration once done
    AUTO_DETECT_DEVICES_CONTINUOUS,
};

// numeric value corresponds to the sample count for the given speed, the enum is just "naming" and avoids expensive lookup :3
enum class CalibrationSpeed {
    FAST = 100,
    SLOW = 250,
    VERY_SLOW = 500,
};

enum class CalibrationError : uint8_t {
    None,
    LackOfRotationalVariance, // move around more
    LackOfTranslationVariance, // move around more
    RmsErrorTooHigh, // the RMS error was too poor to be worth using
    WorseRmsThanLast, // the RMS error was worse than the last calibration attempt
    AxisVarianceTooHigh, // the axis variance was unacceptably high
    WorseAxisVarianceThanLast, // the axis variance was worse than the last calibration attempt
    BadRelativeCalibration, // maths fucked up, try again soz
    Unknown,
};

struct CalibrationErrorMapping {
    CalibrationError eError;
    std::string szLogString;
    std::string szTranslationKey;
};

CalibrationErrorMapping getCalibrationErrorMapping(CalibrationError eCalibrationError);

// An instantaneous pose, SteamVR poses are mapped to this for ease of use with
// Eigen
struct Pose_t {
    Eigen::Matrix3d rot;
    Eigen::Vector3d trans;
    inline Pose_t() { }
    Pose_t(const vr::HmdMatrix34_t hmdMatrix);
    Pose_t(const Eigen::AffineCompact3d& transform);
    Pose_t(const vr::DriverPose_t& driverPose);
};

// An instantaneous sample. Several of these are collected and passed into the
// Calibration algorithm to create a mapping from the reference device to the
// target device.
struct Sample_t {
    Pose_t reference; // what we are calibrating TO
    Pose_t target; // the object that shall be calibrated
    double timestamp = 0; // @FIXME: seems to be unused with existing algorithm?
    bool isPoseValid = false; // whether or not the pose is usable for calibrating. this is filled based on multiple data points in vr::DriverPose_t for accuracy
};

struct CalibrationDevice {
    vr::TrackedDeviceIndex_t deviceId = vr::k_unTrackedDeviceIndexInvalid;
    ipc::protocol::DeviceQuirks_t quirks = ipc::protocol::DeviceQuirks_t::QUIRK_NONE;
    std::string trackingSystem;
    std::string deviceModel;
    std::string deviceSerialNumber;
};

struct CalibrationErrorMetrics {
    TimeSeries<double> rmsError;
    TimeSeries<double> axisIndependence;

    TimeSeries<Eigen::Vector3d> posOffset_rawComputed;
    TimeSeries<Eigen::Vector3d> posOffset_currentCal;
    TimeSeries<Eigen::Vector3d> posOffset_rmsError;

    TimeSeries<double> pose_ref_velocity_mag;
    TimeSeries<double> pose_tgt_velocity_mag;

    TimeSeries<double> computationTime;

    TimeSeries<bool> calibrationApplied;
};

class CalibrationManager;

// An instance of the calibration algorithm, that handles collecting samples, and
// deducing a mapping between calibrations of two tracking systems. It is
// separated from the Calibration Manager to allow one to generate multiple
// calibrations for multiple combinations of playspaces.
class TrackingSystemCalibration {
public:
    void init();
    void start();
    void startContinuous();
    void reset();
    void clearCurrentCalibration();

    // clears samples and updates cosine threshold (for ill-varied sample rejection)
    inline void clearSamples()
    {
        m_samples.clear();
        updateRotationVarianceCosineThreshold();
    }
    // clears metrics data to account for tracking data shifts; should allow the system to escpae from a local minima when a tracking jump occurs to then find the new local minima
    inline void invalidateMetrics()
    {
        m_sampleHistory.clear();
        m_lastRmsError = INFINITY;
        m_lastAxisVariance = 0.0;
    }

    void calibrationTick(const double currentTime);
    void resetCalibrationForDevice(const CalibrationDevice& device); // resets the given device's pose to the raw pose
    void apply(); // applies the calibration to the VR runtime

    // finds the device id given props, if and only if device id is invalid
    void assignTarget(CalibrationDevice& device);
    void tryAssigningTargets();
    // attempts to auto-detect devices
    void autoDetectDevices();
    // marks a device as invalid; space calibrator will attempt reassigning an id next frame to it
    inline void unassignTarget(CalibrationDevice& device)
    {
        device.deviceId = vr::k_unTrackedDeviceIndexInvalid;
    }

    inline const bool devicesAreValid() const
    {
        return referenceDevice.deviceId < vr::k_unMaxTrackedDeviceCount && targetDevice.deviceId < vr::k_unMaxTrackedDeviceCount;
    }

    [[nodiscard]] inline const float getCalibrationProgress() const
    {
        return getSampleCount() == 0 ? 0.0f : (float)((double)m_samples.size() / (double)getSampleCount());
    }

    [[nodiscard]] inline const bool isContinuousCalibration() const
    {
        return state == CalibrationState::CONTINUOUS_IDLE || state == CalibrationState::CONTINUOUS || state == CalibrationState::AUTO_DETECT_DEVICES_CONTINUOUS;
    }

    [[nodiscard]] inline const bool isCalibrating() const
    {
        return state == CalibrationState::START || state == CalibrationState::SAMPLE;
    }

    [[nodiscard]] inline const bool isValidCalibration() const
    {
        return calibrationError == CalibrationError::None;
    }

    inline void forceNextCalibration()
    {
        m_lastRmsError = INFINITY;
        m_lastAxisVariance = 0.0;
    }

public:
    bool isActive = false; // enabled in the UI
    bool hmdIsInReferenceTrackingSystem = false; // whether the hmd is part of the reference device tracking system
    bool isRelativeCalibration = false; // whether the calibration is stored such that its coordinate system is relative to the reference device. this hides tracking anomalies from the reference device and keeps calibrations "valid" for longer
    bool hideContinuousTracker = false;
    bool calibrateMotionVectors = true; // if set to true, will apply calibrations to velocity and acceleration (and angular counterparts)
    bool autoFixPlayspaceJumps = true; // if set to true, attempts to auto-correct playspace jumps that occur
    bool enforceMinimumRotationVariance = true; // if set to true, enforces a minimum rotation variance threshold
    CalibrationError calibrationError = CalibrationError::Unknown; // error state of the last calibration, to be used by ui
    CalibrationState state = CalibrationState::NONE;
    CalibrationSpeed calibrationSpeed = CalibrationSpeed::FAST;

    CalibrationDevice referenceDevice; // what we are calibrating to (ie this will become the ABSOLUTE ORIGIN)
    CalibrationDevice targetDevice; // what we are calibrating (ie this tracking system will be manipulated to match the ref)

    // the calibrated pose
    Eigen::Quaterniond calibratedRotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d calibratedTranslation = Eigen::Vector3d::Zero();
    double calibratedScale = 1.0;

    double wantedUpdateInterval = 1.0;

    CalibrationErrorMetrics errorMetrics = {};

private:
    Sample_t collectSample(double currentTime) const;
    inline size_t getSampleCount() const { return (size_t)calibrationSpeed; }
    [[nodiscard]] inline size_t getMaxSampleHistorySize() const { return (size_t)calibrationSpeed * 5; }

    // a delta sample, used for calibration internal maths
    struct DeltaSample_t {
        bool valid = false;
        Eigen::Vector3d reference;
        Eigen::Vector3d target;
    };

    // math funcs
    inline Eigen::Vector3d axisFromRotationMatrix3(const Eigen::Matrix3d& rot)
    {
        Eigen::AngleAxisd angle_axis(rot);
        return angle_axis.axis() * angle_axis.angle();
    }

    inline double angleFromRotationMatrix3(const Eigen::Matrix3d& rot)
    {
        double trace_val = (rot.trace() - 1.0) / 2.0;
        return acos(std::clamp(trace_val, -1.0, 1.0));
    }

    DeltaSample_t deltaRotationSamples(const Sample_t& s1, const Sample_t& s2);
    Eigen::Quaterniond calibrateRotation(const std::vector<Sample_t>& samples);
    Eigen::Vector3d calibrateTranslation(const std::vector<Sample_t>& samples, const Eigen::Quaterniond& calibratedRotation);
    bool makeCalibrationLocal(Eigen::Quaterniond& rotation, Eigen::Vector3d& translation);
    CalibrationError computeCalibrationOneshot(double currentTime, bool bForceCalibration); // computes instantaneous calibration

    Pose_t applyTransform(const Pose_t& originalPose, const Eigen::AffineCompact3d& transform) const;
    double retargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const;
    Eigen::Vector3d computeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;
    Eigen::Vector4d computeAxisVariance(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation) const;
    bool validateCalibration(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, double& rmsError, Eigen::Vector3d& posOffset, bool isRelative);

    bool checkAndUpdateWorldFromDriver(const CalibrationDevice& device, Eigen::Quaterniond& lastRot, Eigen::Vector3d& lastTrans, Eigen::AffineCompact3d& outDelta) const;
    bool checkWorldFromDriverJump();

    // we collect a series of **VALID** calibrations' worth of samples to improve RMS error accuracy
    void trackCollectedSamplesForErrorTracking();
    bool detectCorrelatingDevices(double currentTime);

    void updateRotationVarianceCosineThreshold();
    bool isSampleVariedEnough(const Sample_t& newSample);

private:
    double m_lastTick = 0.0;
    double m_lastScan = 0.0;

    float m_xTargetPrev = 0.0f;
    float m_yTargetPrev = 0.0f;
    float m_zTargetPrev = 0.0f;

    float m_xRefPrev = 0.0f;
    float m_yRefPrev = 0.0f;
    float m_zRefPrev = 0.0f;

    // @TODO: required? we recompute each time so maybe not even required ig
    double m_lastRmsError = INFINITY;
    double m_lastAxisVariance = 0.0;
    double m_lastSuccessfulCalibTime = 0.0;

    double m_lastNotTrackingTargetWarnTime = 0.0;
    double m_lastNotTrackingRefWarnTime = 0.0;
    double m_lastNoDevicesWarnTime = 0.0;

    // for enforcing rot variance when collecting samples
    // @NOTE: dynamically updated whenever sample count changes
    double m_rotationVarianceCosineThreshold = 0.0;

    // for auto device detection
    int m_candidateScore = 0;
    float m_autoDetectStartTime = NAN;
    vr::TrackedDeviceIndex_t m_candidateRefId = vr::k_unTrackedDeviceIndexInvalid;
    vr::TrackedDeviceIndex_t m_candidateTargetId = vr::k_unTrackedDeviceIndexInvalid;
    double m_autoDetectSpeeds[vr::k_unMaxTrackedDeviceCount] = {};
    bool m_shouldForceCalibrateNextTime = true;

    // for detecting if a device jumps from one playspace to another
    Eigen::Quaterniond m_lastRefWorldFromDriverRot = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond m_lastTargetWorldFromDriverRot = Eigen::Quaterniond::Identity();
    Eigen::Vector3d m_lastRefWorldFromDriverTrans = Eigen::Vector3d::Constant(NAN);
    Eigen::Vector3d m_lastTargetWorldFromDriverTrans = Eigen::Vector3d::Constant(NAN);

public: // @HACK: i just want something in the ui for now
    std::vector<Sample_t> m_samples;

private:
    std::deque<Sample_t> m_sampleHistory; // used ONLY for RMS validation

    friend class CalibrationManager;
};

// Global manager for calibrations, once instance per VR session
class CalibrationManager {
public:
    explicit CalibrationManager();
    void init();
    void start(); // @FIXME: how do i even define a calibration????
    void shutdown();

    void apply(); // applies all calibrations to all devices

    bool loadConfig();
    void saveConfig() const;

    // handles updating pending calibrations every frame
    void calibrationTick(const double currentTime);

    // @TODO: Better getter? idk how a calibration should be defined in terms of ui
    TrackingSystemCalibration& getCalibration(const size_t index);
    size_t getCalibrationCount() const;

    const double getWantedUpdateInterval() const;

    [[nodiscard]] static inline CalibrationManager* getInstance() { return s_instance; }
    [[nodiscard]] inline ipc::IpcClient& getIpcClient() { return m_ipcClient; }

private:
    double m_wantedUpdateInterval = 1.0;
    double m_lastIpcConnectionAttemptTime = 0.0;
    bool m_needToApplyTransformsAfterInit = false;

public:
    vr::DriverPose_t m_poses[vr::k_unMaxTrackedDeviceCount] = {};
    vr::DriverPose_t m_lastPoses[vr::k_unMaxTrackedDeviceCount] = {};

private:
    std::vector<TrackingSystemCalibration> m_calibrations;
    VRState m_vrState;

    ipc::IpcClient m_ipcClient;

    static CalibrationManager* s_instance;
    friend class ::ipc::IpcClient;
    friend class TrackingSystemCalibration;
};
} // namespace spacecal