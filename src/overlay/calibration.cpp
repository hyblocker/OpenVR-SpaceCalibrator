#include "calibration.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "config/configuration_data_versions.h"
#include "configuration.h"
#include "constants.h"
#include "log.h"
#include "openvr.h"
#include "platform.h"
#include "util.h"
#include "vr_core.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <math.h>

namespace spacecal {

CalibrationManager* CalibrationManager::s_instance = nullptr;

// Format mapping table. The rows must be in the exactly same order as Format enum members are defined.
static const CalibrationErrorMapping c_calibrationErrorMapping[] = {
    { CalibrationError::None, "no error", "calibration_error_ok" },
    { CalibrationError::LackOfRotationalVariance, "lack of rotational variance", "calibration_error_lack_of_rotational_variance" },
    { CalibrationError::LackOfTranslationVariance, "lack of translational variance", "calibration_error_lack_of_translational_variance" },
    { CalibrationError::RmsErrorTooHigh, "high RMS error", "calibration_error_rms_error_too_high" },
    { CalibrationError::WorseRmsThanLast, "worse RMS error than the last attempt", "calibration_error_worse_rms_than_last" },
    { CalibrationError::AxisVarianceTooHigh, "axis variance being too high", "calibration_error_axis_variance_too_high" },
    { CalibrationError::WorseAxisVarianceThanLast, "worse axis variance than the last attempt", "calibration_error_worse_axis_variance_than_last" },
    { CalibrationError::BadRelativeCalibration, "bad relative calibration", "calibration_error_bad_relative_calibration" },
};

CalibrationErrorMapping getCalibrationErrorMapping(CalibrationError eCalibrationError)
{
    static_assert(sizeof(c_calibrationErrorMapping) / sizeof(CalibrationErrorMapping) == size_t(CalibrationError::Unknown),
        "The format mapping table doesn't have the correct number of elements");

    const CalibrationErrorMapping mapping = c_calibrationErrorMapping[uint32_t(eCalibrationError)];
    ASSERT(mapping.eError == eCalibrationError, "Format mapping is invalid!");
    return mapping;
}

// 3x4 pose matrix decomposition
Pose_t::Pose_t(const vr::HmdMatrix34_t hmdMatrix)
{
    // [r] [r] [r] [t]
    // [r] [r] [r] [t]
    // [r] [r] [r] [t]
    //    where r is a 3x3 rot matrix, t is a 3 element vector
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot(i, j) = hmdMatrix.m[i][j];
        }
    }
    trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
}

Pose_t::Pose_t(const Eigen::AffineCompact3d& transform)
{
    rot = transform.rotation();
    trans = transform.translation();
}

Pose_t::Pose_t(const vr::DriverPose_t& driverPose)
{
    // apply offsets to get worldspace pose
    Eigen::Quaterniond driverToWorldQ(
        driverPose.qWorldFromDriverRotation.w,
        driverPose.qWorldFromDriverRotation.x,
        driverPose.qWorldFromDriverRotation.y,
        driverPose.qWorldFromDriverRotation.z);
    Eigen::Vector3d driverToWorldV(
        driverPose.vecWorldFromDriverTranslation[0],
        driverPose.vecWorldFromDriverTranslation[1],
        driverPose.vecWorldFromDriverTranslation[2]);

    Eigen::Quaterniond driverRot = driverToWorldQ * Eigen::Quaterniond(driverPose.qRotation.w, driverPose.qRotation.x, driverPose.qRotation.y, driverPose.qRotation.z);

    Eigen::Vector3d driverPos = driverToWorldV + driverToWorldQ * Eigen::Vector3d(driverPose.vecPosition[0], driverPose.vecPosition[1], driverPose.vecPosition[2]);

    Eigen::AffineCompact3d xform = Eigen::Translation3d(driverPos) * driverRot;

    rot = xform.rotation();
    trans = xform.translation();
}

inline vr::HmdQuaternion_t eigenAsVrQuat(const Eigen::Quaterniond& quat)
{
    vr::HmdQuaternion_t q = {
        .w = quat.w(),
        .x = quat.x(),
        .y = quat.y(),
        .z = quat.z(),
    };
    return q;
}

inline vr::HmdVector3_t eigenAsVrVec3(const Eigen::Vector3d& trans)
{
    vr::HmdVector3_t pos = {
        .v = {
            (float)trans.x(),
            (float)trans.y(),
            (float)trans.z(),
        }
    };
    return pos;
}

inline vr::HmdVector3d_t eigenAsVrVec3d(const Eigen::Vector3d& trans)
{
    vr::HmdVector3d_t pos = {
        .v = {
            trans.x(),
            trans.y(),
            trans.z(),
        }
    };
    return pos;
}

TrackingSystemCalibration::DeltaSample_t TrackingSystemCalibration::deltaRotationSamples(const Sample_t& s1, const Sample_t& s2)
{
    // Difference in rotation between samples.
    auto dref = s1.reference.rot * s2.reference.rot.transpose();
    auto dtarget = s1.target.rot * s2.target.rot.transpose();

    // When stuck together, the two tracked objects rotate as a pair,
    // therefore their axes of rotation must be equal between any given pair of samples.
    DeltaSample_t ds = {
        .reference = axisFromRotationMatrix3(dref),
        .target = axisFromRotationMatrix3(dtarget),
    };

    // Reject samples that were too close to each other.
    double refA = angleFromRotationMatrix3(dref);
    double targetA = angleFromRotationMatrix3(dtarget);
    ds.valid = refA > k_ROTATION_ANGLE_THRESHOLD && targetA > k_ROTATION_ANGLE_THRESHOLD && ds.reference.norm() > k_ROTATION_MAGNITUDE_THRESHOLD && ds.target.norm() > k_ROTATION_MAGNITUDE_THRESHOLD;

    ds.reference.normalize();
    ds.target.normalize();
    return ds;
}

Eigen::Quaterniond TrackingSystemCalibration::calibrateRotation(const std::vector<Sample_t>& samples)
{
    std::vector<DeltaSample_t> deltas;

    for (size_t i = 0; i < samples.size(); i++) {
        for (size_t j = 0; j < i; j++) {
            DeltaSample_t delta = deltaRotationSamples(samples[i], samples[j]);
            if (delta.valid)
                deltas.push_back(delta);
        }
    }
    LOG_CALIB_INFO("Got {} samples with {} delta samples", samples.size(), deltas.size());

    if (deltas.size() < k_MIN_DELTA_SAMPLE_COUNT) {
        LOG_CALIB_WARN("Not enough valid delta samples! Aborting calibration...");
        return Eigen::Quaterniond(0, 0, 0, 0);
    }

    // Kabsch algorithm
    // @NOTE: centroid translation is omitted as we're working on rotations ie unit vectors, meaning they're already centered and thus centroid handling is redundant
    Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
    for (size_t i = 0; i < deltas.size(); i++) {
        refPoints.row(i) = deltas[i].reference;
        targetPoints.row(i) = deltas[i].target;
    }

    auto crossCV = refPoints.transpose() * targetPoints;
    auto svd = crossCV.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>();
    if (svd.info() != Eigen::Success) {
        LOG_CALIB_WARN("Failed to compute rotational SVD! Aborting calibration...");
        return Eigen::Quaterniond(0, 0, 0, 0);
    }

    Eigen::Vector3d singularValues = svd.singularValues();
    double conditionNumber = singularValues(1) / singularValues(0);

    if (conditionNumber < k_ROTATION_MIN_VARIANCE) {
        LOG_CALIB_WARN("Not enough rotational variance in collected samples! Got {:.3f}, expected minimum {:.3f} variance. Aborting calibration...", conditionNumber, k_ROTATION_MIN_VARIANCE);
        return Eigen::Quaterniond(0, 0, 0, 0);
    }

    Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
    if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0) {
        i(2, 2) = -1;
    }

    Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
    rot.transposeInPlace();

    Eigen::Quaterniond rotQuat(rot);
    rotQuat.normalize();

    Eigen::Vector3d euler = rot.canonicalEulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);

    LOG_CALIB_INFO("Calibrated rotation (deg): yaw={:.2f} pitch={:.2f} roll={:.2f}", euler[1], euler[2], euler[0]);
    return rotQuat;
}

Eigen::Vector3d TrackingSystemCalibration::calibrateTranslation(const std::vector<Sample_t>& samples, const Eigen::Quaterniond& calibratedRotation)
{
    // @TODO: optimise this, this is REALLY slow -> up to 2 SECOND LAG SPIKE IN DEBUG
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;
    deltas.reserve(samples.size() * (samples.size() - 1)); // combination(2, 1)

    // rotation is only applied to the target tracking system, as that's the tracking system the calibration is targeting! we do not need to modify the reference pose!
    for (size_t i = 0; i < samples.size(); i++) {
        Sample_t sample_i = samples[i];
        sample_i.target.rot = calibratedRotation * sample_i.target.rot;
        sample_i.target.trans = calibratedRotation * sample_i.target.trans;

        for (size_t j = 0; j < i; j++) {
            Sample_t sample_j = samples[j];
            sample_j.target.rot = calibratedRotation * sample_j.target.rot;
            sample_j.target.trans = calibratedRotation * sample_j.target.trans;

            auto QAi = sample_i.reference.rot.transpose();
            auto QAj = sample_j.reference.rot.transpose();
            auto dQA = QAj - QAi;
            auto CA = QAj * (sample_j.reference.trans - sample_j.target.trans) - QAi * (sample_i.reference.trans - sample_i.target.trans);
            deltas.push_back(std::make_pair(CA, dQA));

            auto QBi = sample_i.target.rot.transpose();
            auto QBj = sample_j.target.rot.transpose();
            auto dQB = QBj - QBi;
            auto CB = QBj * (sample_j.reference.trans - sample_j.target.trans) - QBi * (sample_i.reference.trans - sample_i.target.trans);
            deltas.push_back(std::make_pair(CB, dQB));
        }
    }

    Eigen::VectorXd constants(deltas.size() * 3);
    Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

    for (size_t i = 0; i < deltas.size(); i++) {
        for (int axis = 0; axis < 3; axis++) {
            constants(i * 3 + axis) = deltas[i].first(axis);
            coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
        }
    }

    auto svd = coefficients.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>();

    if (svd.info() != Eigen::Success) {
        LOG_CALIB_WARN("Failed to compute numerically stable translation SVD! Aborting calibration...");
        return Eigen::Vector3d(NAN, NAN, NAN);
    }

    Eigen::Vector3d singularValues = svd.singularValues();

    // min singular value represents how varied the motion is
    double motionVariance = singularValues(2);
    // condition number represents the noise factor, and is ratio of max to min
    double amplifiedNoiseFactor = singularValues(0) / motionVariance;

    LOG_CALIB_INFO("Translation motion variance: {:.4f}, amplified noise factor: {:.2f}x", motionVariance, amplifiedNoiseFactor);

    // we want to only accept well-conditioned solutions, to minimise the discrepenancy of the error term
    if (motionVariance < k_TRANSLATION_MIN_MOTION_THRESHOLD || amplifiedNoiseFactor > k_TRANSLATION_MAX_AMPLIFIED_NOISE_FACTOR) {
        return Eigen::Vector3d(NAN, NAN, NAN);
    }

    Eigen::Vector3d trans = svd.solve(constants);

    LOG_CALIB_INFO("Calibrated translation (cm): x={:.2f} y={:.2f} z={:.2f}", trans[0] * 100.0, trans[1] * 100.0, trans[2] * 100.0);
    return trans;
}

void TrackingSystemCalibration::trackCollectedSamplesForErrorTracking()
{
    if (m_samples.empty())
        return;

    // m_sampleHistory is sort of like a ring buffer here, we use it to keep track of 5 calibrations' worth of samples, we append here
    // and keep the size constrained to keep speed and recency bias acceptable
    m_sampleHistory.insert(m_sampleHistory.end(), m_samples.begin(), m_samples.end());

    if (m_sampleHistory.size() > getMaxSampleHistorySize()) {
        size_t numElementsToRemove = m_sampleHistory.size() - getMaxSampleHistorySize();
        m_sampleHistory.erase(m_sampleHistory.begin(), m_sampleHistory.begin() + numElementsToRemove);
    }
}

CalibrationError TrackingSystemCalibration::computeCalibrationOneshot(double currentTime, bool bForceCalibration)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    // @TODO: do NOT apply until we are certain this is a better calibration.

    // we clear calibration history if the calibration was invalid for a long time. this is to prevent a deadlock.
    // say im using continuous and i took my hmd off, and when i came back it recentered. the calibration history would be invalid and thus we want to invalidate it.
    if (m_lastSuccessfulCalibTime > 0.0 && (currentTime - m_lastSuccessfulCalibTime) > k_MAX_INVALID_CALIBRATION_TIME_SEC) {
        LOG_CALIB_INFO("Clearing calibration history due to old calibration data expiring.");
        m_sampleHistory.clear();
    }

    // apply samples to avoid sampling twice
    Eigen::Quaterniond computedRotation = calibrateRotation(m_samples);
    Eigen::Vector3d computedTranslation = calibrateTranslation(m_samples, computedRotation);

    CalibrationError eCalibrationError = CalibrationError::None;
    // ensure rotation is valid
    if (computedRotation.squaredNorm() < 1e-6) {
        eCalibrationError = CalibrationError::LackOfRotationalVariance;
    }

    // ensure translation is valid
    if (eCalibrationError == CalibrationError::None && (isnan(computedTranslation.x()) || isnan(computedTranslation.y()) || isnan(computedTranslation.z()))) {
        eCalibrationError = CalibrationError::LackOfTranslationVariance;
    }

    if (isContinuousCalibration()) {
        // re-compute the rms error for the current sample set, to ensure we keep state in sync with reality
        // this handles the case of a hmd drifting over time
        // this also handles the case of recentering (eg assume quest + vive trackers, i take headset off to do something, i come back, headset auto recenters invalidating calibration so we must reject the old calibration)
        Eigen::Vector3d _posOffset_temp = Eigen::Vector3d::Zero();
        validateCalibration(calibratedRotation, calibratedTranslation, m_lastRmsError, _posOffset_temp, isRelativeCalibration);
    }

    double rmsError = 0.0;
    Eigen::Vector3d posOffset = Eigen::Vector3d::Zero();
    if (eCalibrationError == CalibrationError::None && (!validateCalibration(computedRotation, computedTranslation, rmsError, posOffset, false))) {
        eCalibrationError = CalibrationError::RmsErrorTooHigh;
    }

    if (eCalibrationError == CalibrationError::None && (isContinuousCalibration() && rmsError > m_lastRmsError)) {
        eCalibrationError = CalibrationError::WorseRmsThanLast;
    }

    double axisVariance = computeAxisVariance(computedRotation, computedTranslation)(1);
    if (eCalibrationError == CalibrationError::None && axisVariance < k_MAX_AXIS_VARIANCE_THRESHOLD) {
        eCalibrationError = CalibrationError::AxisVarianceTooHigh;
    }

    if (eCalibrationError == CalibrationError::None && axisVariance < m_lastAxisVariance) {
        eCalibrationError = CalibrationError::WorseAxisVarianceThanLast;
    }

    if (eCalibrationError == CalibrationError::None && (isRelativeCalibration && !makeCalibrationLocal(computedRotation, computedTranslation))) {
        eCalibrationError = CalibrationError::BadRelativeCalibration;
    }

    // @TODO: calibration metrics
    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    bool bApplied = (eCalibrationError == CalibrationError::None || bForceCalibration);

    errorMetrics.rmsError.push(currentTime, k_METRIC_HISTORY_TIMESPAN, rmsError);
    errorMetrics.axisIndependence.push(currentTime, k_METRIC_HISTORY_TIMESPAN, axisVariance);
    errorMetrics.calibrationApplied.push(currentTime, k_METRIC_HISTORY_TIMESPAN, bApplied);
    errorMetrics.computationTime.push(currentTime, k_METRIC_HISTORY_TIMESPAN, elapsedMs);
    errorMetrics.posOffset_rmsError.push(currentTime, k_METRIC_HISTORY_TIMESPAN, posOffset);
    errorMetrics.posOffset_rawComputed.push(currentTime, k_METRIC_HISTORY_TIMESPAN, computedTranslation);
    errorMetrics.posOffset_currentCal.push(currentTime, k_METRIC_HISTORY_TIMESPAN, bApplied ? computedTranslation : calibratedTranslation);

    if (eCalibrationError == CalibrationError::None || bForceCalibration) {
        calibrationError = bForceCalibration ? CalibrationError::None : eCalibrationError;
        calibratedRotation = computedRotation;
        calibratedTranslation = computedTranslation;

        m_lastRmsError = rmsError;

        if (eCalibrationError == CalibrationError::None && isContinuousCalibration()) {
            trackCollectedSamplesForErrorTracking();
            m_lastSuccessfulCalibTime = currentTime;
        }

        apply();
        CalibrationManager::getInstance()->saveConfig();
    } else {
        // @TODO: propagate rejection reason to UI to provide user with feedback on how to improve calibration
        std::string calibErrString = getCalibrationErrorMapping(eCalibrationError).szLogString;
        LOG_CALIB_WARN("Rejecting calibration due to {}; RMS: {}", calibErrString, rmsError);
    }

    return eCalibrationError;
}

bool TrackingSystemCalibration::makeCalibrationLocal(Eigen::Quaterniond& rotation, Eigen::Vector3d& translation)
{
    if (isRelativeCalibration && !m_samples.empty()) {
        Sample_t& lastSample = m_samples.back();
        // take calibration as a worldspace transformation matrix; take its inverse and apply it to the last known worldspace pose of the ref device
        Eigen::Affine3d worldCalib = Eigen::Translation3d(translation) * rotation;
        Eigen::Affine3d refPose = Eigen::Translation3d(lastSample.reference.trans) * lastSample.reference.rot;
        Eigen::Affine3d targetPose = Eigen::Translation3d(lastSample.target.trans) * lastSample.target.rot;

        // We estimate C_L, a matrix mapping the tracker to it's position in HMD space relative to the HMD.
        // To do this we apply the calibration and then make it relative to the HMD.

        // T_W = C * T
        Eigen::Affine3d calibratedTarget = worldCalib * targetPose;

        // C_L = H^-1 * T_W
        Eigen::Affine3d localCalib = refPose.inverse() * calibratedTarget;

        rotation = Eigen::Quaterniond(localCalib.rotation());
        translation = localCalib.translation();

        Eigen::Vector3d euler = rotation.toRotationMatrix().canonicalEulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);
        LOG_CALIB_INFO("Converted to local space. rotation (deg): yaw={:.2f} pitch={:.2f} roll={:.2f} ; translation (cm): x={:.2f} y={:.2f} z={:.2f}",
            euler[1], euler[2], euler[0],
            translation[0] * 100.0, translation[1] * 100.0, translation[2] * 100.0);
        return true;
    }
    return false;
}

Pose_t TrackingSystemCalibration::applyTransform(const Pose_t& originalPose, const Eigen::AffineCompact3d& transform) const
{
    Pose_t pose(originalPose);
    pose.rot = transform.rotation() * pose.rot;
    pose.trans = transform * pose.trans;
    return pose;
}

double TrackingSystemCalibration::retargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const
{
    double errorAccum = 0;
    int sampleCount = 0;

    // we accumulate samples for the sample history + these samples, to hopefully have a trustworthy error term
    for (auto& sample : m_sampleHistory) {
        if (!sample.isPoseValid)
            continue;

        // Apply transformation
        const auto updatedPose = applyTransform(sample.target, calibration);

        const Eigen::Vector3d hmdPoseSpace = sample.reference.rot * hmdToTargetPos + sample.reference.trans;

        // Compute error term
        double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
        errorAccum += error;
        sampleCount++;
    }

    for (auto& sample : m_samples) {
        if (!sample.isPoseValid)
            continue;

        // Apply transformation
        const auto updatedPose = applyTransform(sample.target, calibration);

        const Eigen::Vector3d hmdPoseSpace = sample.reference.rot * hmdToTargetPos + sample.reference.trans;

        // Compute error term
        double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
        errorAccum += error;
        sampleCount++;
    }

    return sqrt(errorAccum / sampleCount);
}

Eigen::Vector3d TrackingSystemCalibration::computeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const
{
    Eigen::Vector3d accum = Eigen::Vector3d::Zero();
    int sampleCount = 0;

    // we accumulate samples for the sample history + these samples, to hopefully have a trustworthy error term
    for (auto& sample : m_sampleHistory) {
        if (!sample.isPoseValid)
            continue;

        // Apply transformation
        const auto updatedPose = applyTransform(sample.target, calibration);

        // Now move the transform from world to HMD space
        const auto hmdOriginPos = updatedPose.trans - sample.reference.trans;
        const auto hmdSpace = sample.reference.rot.inverse() * hmdOriginPos;

        accum += hmdSpace;
        sampleCount++;
    }

    for (auto& sample : m_samples) {
        if (!sample.isPoseValid)
            continue;

        // Apply transformation
        const auto updatedPose = applyTransform(sample.target, calibration);

        // Now move the transform from world to HMD space
        const auto hmdOriginPos = updatedPose.trans - sample.reference.trans;
        const auto hmdSpace = sample.reference.rot.inverse() * hmdOriginPos;

        accum += hmdSpace;
        sampleCount++;
    }

    accum /= sampleCount;

    return accum;
}

// taken from legacy codebase
Eigen::Vector4d TrackingSystemCalibration::computeAxisVariance(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation) const
{
    // We want to determine if the user rotated in enough axis to find a unique solution.
    // It's sufficient to rotate in two axis - this is because once we constrain the mapping
    // of those two orthogonal basis vectors, the third is determined by the cross product of
    // those two basis vectors. So, the question we then have to answer is - after accounting for
    // translational movement of the HMD itself, are we too close to having only moved on a plane?

    // To determine this, we perform primary component analysis on the rotation quaternions themselves.
    // Since an angle axis quaternion is defined as the sum of Qidentity*cos(angle/2) + Qaxis*sin(angle/2),
    // we expect that rotations around a single axis will have two primary components: One corresponding
    // to the identity component, and one to the axis component. Thus, we check the variance (eigenvalue) of
    // the third primary component to see if we've moved in two axis.
    Eigen::Affine3d calibration = Eigen::Translation3d(translation) * rotation;

    std::ostringstream dbgStream;

    std::vector<Eigen::Vector4d> points;

    Eigen::Vector4d mean = Eigen::Vector4d::Zero();

    for (auto& sample : m_samples) {
        if (!sample.isPoseValid)
            continue;

        auto q = Eigen::Quaterniond(sample.target.rot);
        auto point = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
        mean += point;

        points.push_back(point);
    }
    mean /= (double)points.size();

    // Compute covariance matrix
    Eigen::Matrix4d covMatrix = Eigen::Matrix4d::Zero();

    for (auto& point : points) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                covMatrix(i, j) += (point(i) - mean(i)) * (point(j) - mean(j));
            }
        }
    }
    covMatrix /= (double)points.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
    solver.compute(covMatrix);

    return solver.eigenvalues();
}

bool TrackingSystemCalibration::validateCalibration(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation, double& rmsError, Eigen::Vector3d& posOffset, bool isRelative)
{
    bool ok = true;

    Eigen::Affine3d calibrationMatrix = Eigen::Translation3d(translation) * rotation;

    // recompute world space calibration from relative calibration
    if (isRelative) {
        Eigen::Vector3d sampleCalibTrans = Eigen::Vector3d::Zero();
        Eigen::Quaterniond sampleCalibRot = Eigen::Quaterniond::Identity();
        size_t validCount = 0;

        for (auto& sample : m_samples) {
            if (!sample.isPoseValid)
                continue;

            // compute H' and T'
            Eigen::Affine3d refWorldNow = Eigen::Translation3d(sample.reference.trans) * sample.reference.rot;
            Eigen::Affine3d targetWorldNow = Eigen::Translation3d(sample.target.trans) * sample.target.rot;

            // T_H = H' * C_L
            Eigen::Affine3d expectedTargetWorld = refWorldNow * calibrationMatrix;

            // C' = T_H * (T')^-1
            Eigen::Affine3d worldCalib = expectedTargetWorld * targetWorldNow.inverse();

            // averaging
            sampleCalibTrans += worldCalib.translation();
            Eigen::Quaterniond q(worldCalib.linear());
            if (validCount == 0) {
                sampleCalibRot = q;
            } else {
                sampleCalibRot = sampleCalibRot.slerp(1.0 / (validCount + 1), q);
            }
            validCount++;
        }

        if (validCount > 0) {
            calibrationMatrix = Eigen::Translation3d(sampleCalibTrans / static_cast<double>(validCount)) * sampleCalibRot;
        }
    }

    posOffset = computeRefToTargetOffset(calibrationMatrix);
    rmsError = retargetingErrorRMS(posOffset, calibrationMatrix);
    if (rmsError > k_MAX_RETARGETING_RMS_ERROR_THRESHOLD)
        ok = false;

    return ok;
}

Sample_t TrackingSystemCalibration::collectSample(double currentTime) const
{
    vr::DriverPose_t reference, target;
    reference.poseIsValid = false;
    target.poseIsValid = false;

    bool bIsTrackingOk = true;

    if (referenceDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
        bool actuallyConnected = VRState::getInstance()->getVrDevice(referenceDevice.deviceId).bIsConnected;
        reference = CalibrationManager::getInstance()->m_poses[this->referenceDevice.deviceId];
        if (!(actuallyConnected && reference.poseIsValid && reference.result == vr::ETrackingResult::TrackingResult_Running_OK)) {
            // dont spam logs
            if (actuallyConnected) {
                LOG_CALIB_WARN("Reference device is not tracking");
            }
            bIsTrackingOk = false;
        }
    }
    if (targetDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
        bool actuallyConnected = VRState::getInstance()->getVrDevice(targetDevice.deviceId).bIsConnected;
        target = CalibrationManager::getInstance()->m_poses[this->targetDevice.deviceId];
        if (!(actuallyConnected && target.poseIsValid && target.result == vr::ETrackingResult::TrackingResult_Running_OK)) {
            // dont spam logs
            if (actuallyConnected) {
                LOG_CALIB_WARN("Target device is not tracking");
            }
            bIsTrackingOk = false;
        }
    }

    if (!bIsTrackingOk) {
        return Sample_t { .isPoseValid = bIsTrackingOk };
    }

    return Sample_t {
        .reference = Pose_t(reference),
        .target = Pose_t(target),
        .timestamp = currentTime,
        .isPoseValid = bIsTrackingOk
    };
}

bool TrackingSystemCalibration::checkAndUpdateWorldFromDriver(const CalibrationDevice& device, Eigen::Quaterniond& lastRot, Eigen::Vector3d& lastTrans, Eigen::AffineCompact3d& outDelta) const
{
    if (device.deviceId >= vr::k_unMaxTrackedDeviceCount)
        return false;

    const auto& pose = CalibrationManager::getInstance()->m_poses[device.deviceId];
    bool actuallyConnected = VRState::getInstance()->getVrDevice(device.deviceId).bIsConnected;
    if (!VRState::getInstance()->getVrDevice(device.deviceId).bIsConnected || !actuallyConnected || !pose.poseIsValid)
        return false;

    Eigen::Quaterniond curRot(pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
    Eigen::Vector3d curTrans(pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]);

    if (lastTrans.hasNaN()) {
        lastRot = curRot;
        lastTrans = curTrans;
        return false;
    }

    double angleDelta = curRot.angularDistance(lastRot);
    double posDelta = (curTrans - lastTrans).norm();
    bool changed = (angleDelta > k_PLAYSPACE_JUMP_WORLD_FROM_DRIVER_ANGLE_THRESHOLD_RAD) || (posDelta > k_PLAYSPACE_JUMP_WORLD_FROM_DRIVER_POS_THRESHOLD_METERS);

    if (changed) {
        Eigen::AffineCompact3d oldWFD = Eigen::Translation3d(lastTrans) * lastRot;
        Eigen::AffineCompact3d newWFD = Eigen::Translation3d(curTrans) * curRot;
        outDelta = newWFD * oldWFD.inverse();
    }

    lastRot = curRot;
    lastTrans = curTrans;
    return changed;
}

bool TrackingSystemCalibration::checkWorldFromDriverJump()
{
    Eigen::AffineCompact3d deltaRef = Eigen::AffineCompact3d::Identity();
    Eigen::AffineCompact3d deltaTarget = Eigen::AffineCompact3d::Identity();

    bool refChanged = checkAndUpdateWorldFromDriver(referenceDevice, m_lastRefWorldFromDriverRot, m_lastRefWorldFromDriverTrans, deltaRef);
    bool targetChanged = checkAndUpdateWorldFromDriver(targetDevice, m_lastTargetWorldFromDriverRot, m_lastTargetWorldFromDriverTrans, deltaTarget);

    if (!refChanged && !targetChanged) {
        return false;
    }

    LOG_CALIB_WARN("Detected tracking discontinuity (ref={} target={}), attempting to recover calibration...", refChanged, targetChanged);

    // clear sample data as it'll have invalid data since we detected this discontinuity
    m_sampleHistory.clear();
    m_samples.clear();
    m_lastRmsError = INFINITY;
    m_lastAxisVariance = 0.0;

    // attempt to recover the calibration so that the user doesn't notice
    Eigen::AffineCompact3d calib = Eigen::Translation3d(calibratedTranslation) * calibratedRotation;
    if (targetChanged) {
        calib = deltaTarget.inverse() * calib;
    }
    if (refChanged) {
        calib = deltaRef * calib;
    }

    calibratedRotation = Eigen::Quaterniond(calib.rotation()).normalized();
    calibratedTranslation = calib.translation();

    return true;
}

void TrackingSystemCalibration::init()
{
    m_samples.reserve((size_t)CalibrationSpeed::VERY_SLOW);
    updateRotationVarianceCosineThreshold();
}

void TrackingSystemCalibration::reset()
{
    LOG_CALIB_INFO("Resetting calibration...");
    m_samples.clear();
    m_sampleHistory.clear();
    // @NOTE: should we do this? need to compare with live behaviour
    calibratedRotation = Eigen::Quaterniond::Identity();
    calibratedTranslation = Eigen::Vector3d::Zero();
    calibratedScale = 1.0;

    m_lastRmsError = INFINITY;
    m_lastAxisVariance = 0.0;
    m_lastSuccessfulCalibTime = 0.0;

    m_lastRefWorldFromDriverTrans = Eigen::Vector3d::Constant(NAN);
    m_lastTargetWorldFromDriverTrans = Eigen::Vector3d::Constant(NAN);

    updateRotationVarianceCosineThreshold();
}

void TrackingSystemCalibration::start()
{
    reset();
    state = CalibrationState::START;
    wantedUpdateInterval = 0.0;
    assignTarget(referenceDevice);
    assignTarget(targetDevice);

    // update state
    const auto& hmdDevice = VRState::getInstance()->getVrDevice(vr::k_unTrackedDeviceIndex_Hmd);
    hmdIsInReferenceTrackingSystem = hmdDevice.szTrackingSystemId == referenceDevice.trackingSystem;
}

void TrackingSystemCalibration::startContinuous()
{
    reset();
    state = CalibrationState::CONTINUOUS_IDLE;
    wantedUpdateInterval = 0.0;
    assignTarget(referenceDevice);
    assignTarget(targetDevice);

    // update state
    const auto& hmdDevice = VRState::getInstance()->getVrDevice(vr::k_unTrackedDeviceIndex_Hmd);
    hmdIsInReferenceTrackingSystem = hmdDevice.szTrackingSystemId == referenceDevice.trackingSystem;
}

void TrackingSystemCalibration::updateRotationVarianceCosineThreshold()
{
    const double range_rad = k_ROTATION_VARIANCE_TARGET_DEGREES * (M_PI / 180.0);
    const double min_angle_rad = range_rad / (1.2 * cbrt((double)getSampleCount()));
    m_rotationVarianceCosineThreshold = cos(min_angle_rad / 2.0);
}

bool TrackingSystemCalibration::isSampleVariedEnough(const Sample_t& newSample)
{
    if (!enforceMinimumRotationVariance || m_samples.empty())
        return true;

    const Eigen::Quaterniond new_rotation(newSample.reference.rot);

    for (const auto& sample : m_samples) {
        const Eigen::Quaterniond existing_rotation(sample.reference.rot);
        if (std::abs(new_rotation.dot(existing_rotation)) >= m_rotationVarianceCosineThreshold) {
            return false;
        }
    }

    return true;
}

void TrackingSystemCalibration::calibrationTick(const double currentTime)
{
    if (!vr::VRSystem())
        return;

    if ((currentTime - m_lastTick) < (1.0 / k_TICK_RATE_HZ))
        return;

    // original code checks hmd specifically, we should check that the hmd and ref arent at 0 0 0
    // check that ref is not at origin
    if (hmdIsInReferenceTrackingSystem) {
        if (targetDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
            auto targetPose = CalibrationManager::getInstance()->m_poses[targetDevice.deviceId].vecPosition;
            if ((targetPose[0] == 0.0 && targetPose[1] == 0.0 && targetPose[2] == 0.0) || (m_xTargetPrev == targetPose[0] && m_yTargetPrev == targetPose[1] && m_zTargetPrev == targetPose[2])) {
                if (currentTime - m_lastNotTrackingTargetWarnTime >= k_WARN_DEVICE_NOT_TRACKING_INTERVAL_SEC) {
                    LOG_CALIB_WARN("Target device tracking didn't update, skipping update");
                    m_lastNotTrackingTargetWarnTime = currentTime;
                }
                return;
            }
        }
        if (referenceDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
            auto refPose = CalibrationManager::getInstance()->m_poses[referenceDevice.deviceId].vecPosition;
            if ((refPose[0] == 0.0 && refPose[1] == 0.0 && refPose[2] == 0.0) || (m_xRefPrev == refPose[0] && m_yRefPrev == refPose[1] && m_zRefPrev == refPose[2])) {
                if (currentTime - m_lastNotTrackingRefWarnTime >= k_WARN_DEVICE_NOT_TRACKING_INTERVAL_SEC) {
                    LOG_CALIB_WARN("Reference device tracking didn't update, skipping update");
                    m_lastNotTrackingRefWarnTime = currentTime;
                }
                return;
            }
        }
    }

    // detect playspace jumps and try auto-correcting for it.
    if (autoFixPlayspaceJumps) {
        if (checkWorldFromDriverJump()) {
            apply();
        }
    }

    if (hmdIsInReferenceTrackingSystem) {
        if (targetDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
            auto targetPose = CalibrationManager::getInstance()->m_poses[targetDevice.deviceId].vecPosition;
            m_xTargetPrev = (float)targetPose[0];
            m_yTargetPrev = (float)targetPose[1];
            m_zTargetPrev = (float)targetPose[2];
        }
        if (referenceDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
            auto refPose = CalibrationManager::getInstance()->m_poses[referenceDevice.deviceId].vecPosition;
            m_xRefPrev = (float)refPose[0];
            m_yRefPrev = (float)refPose[1];
            m_zRefPrev = (float)refPose[2];
        }
    }

    if (state == CalibrationState::AUTO_DETECT_DEVICES_STANDARD || state == CalibrationState::AUTO_DETECT_DEVICES_CONTINUOUS) {
        // @TODO: measure velocities, compare, determine devices
        if (detectCorrelatingDevices(currentTime)) {
            if (state == CalibrationState::AUTO_DETECT_DEVICES_STANDARD) {
                state = CalibrationState::NONE;
            } else if (state == CalibrationState::AUTO_DETECT_DEVICES_CONTINUOUS) {
                state = CalibrationState::CONTINUOUS_IDLE;
            }
        }
    }

    m_lastTick = currentTime;

    if (state == CalibrationState::CONTINUOUS_IDLE) {
        wantedUpdateInterval = 1.0;
        if ((currentTime - m_lastScan) >= 1.0) {
            assignTarget(referenceDevice);
            assignTarget(targetDevice);
            const auto& hmdDevice = VRState::getInstance()->getVrDevice(vr::k_unTrackedDeviceIndex_Hmd);
            hmdIsInReferenceTrackingSystem = hmdDevice.szTrackingSystemId == referenceDevice.trackingSystem;
            m_lastScan = currentTime;
        }
    }

    if (state == CalibrationState::NONE) {
        wantedUpdateInterval = 1.0;
        if ((currentTime - m_lastScan) >= 1.0) {
            assignTarget(referenceDevice);
            assignTarget(targetDevice);
            m_lastScan = currentTime;
        }
        return;
    }

    if (state == CalibrationState::EDITING) {
        wantedUpdateInterval = 0.1;
        if ((currentTime - m_lastScan) >= 0.1) {
            assignTarget(referenceDevice);
            assignTarget(targetDevice);
            m_lastScan = currentTime;
        }
        return;
    }

    if (state == CalibrationState::START || state == CalibrationState::CONTINUOUS_IDLE) {
        bool ok = true;

        bool bLogCalibrationState = false;

        if (currentTime - m_lastNoDevicesWarnTime >= k_WARN_DEVICE_NOT_TRACKING_INTERVAL_SEC) {
            if (state == CalibrationState::CONTINUOUS_IDLE) {
                LOG_CALIB_INFO("Beginning continuous calibration...");
            } else {
                LOG_CALIB_INFO("Beginning calibration...");
            }
            LOG_CALIB_INFO("  Reference device: ID: {}, tracking system: {}, model: {} serial: {}", referenceDevice.deviceId, referenceDevice.trackingSystem, referenceDevice.deviceModel, referenceDevice.deviceSerialNumber);
            LOG_CALIB_INFO("  Target device: ID: {}, tracking system: {}, model: {} serial: {}", targetDevice.deviceId, targetDevice.trackingSystem, targetDevice.deviceModel, targetDevice.deviceSerialNumber);
            m_lastNoDevicesWarnTime = currentTime;
            bLogCalibrationState = true;
        }

        if (referenceDevice.deviceId >= vr::k_unMaxTrackedDeviceCount) {
            if (bLogCalibrationState) {
                LOG_CALIB_ERROR("Missing reference device");
            }
            ok = false;
        }
        if (targetDevice.deviceId >= vr::k_unMaxTrackedDeviceCount) {
            if (bLogCalibrationState) {
                LOG_CALIB_ERROR("Missing target device");
            }
            ok = false;
        }

        Sample_t sample = collectSample(currentTime);
        if (!sample.isPoseValid) {
            ok = false;
        }

        if (!ok) {
            if (state == CalibrationState::START)
                state = CalibrationState::NONE;
            if (bLogCalibrationState) {
                LOG_CALIB_ERROR("Aborting calibration!");
            }
            return;
        }

        resetCalibrationForDevice(targetDevice);
        if (state == CalibrationState::CONTINUOUS_IDLE) {
            state = CalibrationState::CONTINUOUS;
            LOG_CALIB_INFO("Starting continuous calibration...");
        } else {
            state = CalibrationState::SAMPLE;
            LOG_CALIB_INFO("Starting calibration...");
        }
        wantedUpdateInterval = 0.0;

        return;
    }

    // prevent overfitting with possibly poor data
    if (m_samples.size() < getSampleCount()) {
        auto sample = collectSample(currentTime);
        if (sample.isPoseValid && isSampleVariedEnough(sample)) {
            // we want to completely ignore poor samples during calibration
            m_samples.push_back(sample);

            // metrics: velocity
            // magnitude of velocity of ref and tgt
            if (m_samples.size() >= 2) {
                const Sample_t& currentFrame = m_samples.back();
                size_t stride = std::min(static_cast<size_t>(10), m_samples.size() - 1);
                const Sample_t& previousFrame = m_samples[m_samples.size() - 1 - stride];
                double dt = currentFrame.timestamp - previousFrame.timestamp;

                // epsilon
                if (dt > 1e-5) {
                    Eigen::Vector3d refDeltaTrans = currentFrame.reference.trans - previousFrame.reference.trans;
                    double refVelocityMag = refDeltaTrans.norm() / dt;
                    Eigen::Vector3d tgtDeltaTrans = currentFrame.target.trans - previousFrame.target.trans;
                    double tgtVelocityMag = tgtDeltaTrans.norm() / dt;
                    errorMetrics.pose_ref_velocity_mag.push(currentTime, k_METRIC_HISTORY_TIMESPAN, refVelocityMag);
                    errorMetrics.pose_tgt_velocity_mag.push(currentTime, k_METRIC_HISTORY_TIMESPAN, tgtVelocityMag);
                }
            }
        }
    }

    if (m_samples.size() == getSampleCount()) {
        switch (state) {
        case CalibrationState::SAMPLE: {
            // force calibration in standard mode
            if (computeCalibrationOneshot(currentTime, true) == CalibrationError::None) {
                LOG_CALIB_INFO("Finished calibration, profile saved");
            }

            LOG_CALIB_INFO("Finished standard calibration, profile saved");

            state = CalibrationState::NONE;
            break;
        }
        case CalibrationState::CONTINUOUS: {
            if (computeCalibrationOneshot(currentTime, false) == CalibrationError::None) {
                LOG_CALIB_INFO("Finished continuous calibration, profile saved");
            }

            // clear half the samples for the next calibration
            size_t dwEraseSampleSize = getSampleCount() / 2;
            m_samples.erase(m_samples.begin(), m_samples.begin() + dwEraseSampleSize);

            // state should remain as continuous
            break;
        }
        default:
            break;
        }
    }
}

void TrackingSystemCalibration::assignTarget(CalibrationDevice& device)
{
    // prevent stale state, check if the device is valid and wasnt disconnected, else reset and find it again (may not work!)
    if (device.deviceId < vr::k_unMaxTrackedDeviceCount) {
        const auto& currentDev = VRState::getInstance()->getVrDevice(device.deviceId);

        if (!currentDev.bIsConnected || (!device.deviceSerialNumber.empty() && currentDev.szSerial != device.deviceSerialNumber)) {
            device.deviceId = vr::k_unTrackedDeviceIndexInvalid;
        }
    }

    if (device.deviceId == vr::k_unTrackedDeviceIndexInvalid) {
        auto theDevice = VRState::getInstance()->findVrDevice(device.trackingSystem, device.deviceModel, device.deviceSerialNumber);
        if (theDevice.bIsConnected && theDevice.dwDeviceIndex < vr::k_unMaxTrackedDeviceCount) {
            LOG_CALIB_INFO("Assigning device from tracking system {}; model: {} SN: ({})...", device.trackingSystem, device.deviceModel, device.deviceSerialNumber);
            device.deviceId = theDevice.dwDeviceIndex;
        }
    }
}

void TrackingSystemCalibration::tryAssigningTargets()
{
    if (!devicesAreValid()) {
        // successfully loaded calibration; update runtime state
        const auto& hmdDevice = VRState::getInstance()->getVrDevice(vr::k_unTrackedDeviceIndex_Hmd);

        assignTarget(referenceDevice);
        assignTarget(targetDevice);

        // update state
        hmdIsInReferenceTrackingSystem = hmdDevice.szTrackingSystemId == referenceDevice.trackingSystem;
    }
}

void TrackingSystemCalibration::autoDetectDevices()
{
    LOG_CALIB_INFO("Entering auto-detection mode...");
    state = isContinuousCalibration() ? CalibrationState::AUTO_DETECT_DEVICES_CONTINUOUS : CalibrationState::AUTO_DETECT_DEVICES_STANDARD;
    m_candidateScore = 0;
    m_autoDetectStartTime = NAN;
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        m_autoDetectSpeeds[i] = 0.0;
    }
}

bool TrackingSystemCalibration::detectCorrelatingDevices(double currentTime)
{
    // init time (NAN = new auto detect run)
    if (isnan(m_autoDetectStartTime)) {
        m_autoDetectStartTime = (float)currentTime;
    }

    vr::TrackedDeviceIndex_t frameRefId = vr::k_unTrackedDeviceIndexInvalid;
    vr::TrackedDeviceIndex_t frameTargetId = vr::k_unTrackedDeviceIndexInvalid;
    double frameClosestSpeedDelta = 999.0;

    // because some vendors (vd) do njot provide velocity we estimate it from first principles...
    double deltaTime = currentTime - m_lastTick;
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        const auto& pose = CalibrationManager::getInstance()->m_poses[i];
        bool actuallyConnected = VRState::getInstance()->getVrDevice(i).bIsConnected;
        const auto& lastPose = CalibrationManager::getInstance()->m_lastPoses[i];
        if (!actuallyConnected || !pose.poseIsValid || pose.result != vr::ETrackingResult::TrackingResult_Running_OK)
            continue;
        if (!lastPose.deviceIsConnected || !lastPose.poseIsValid || lastPose.result != vr::ETrackingResult::TrackingResult_Running_OK)
            continue;
        double deltas[3] = { (pose.vecPosition[0] - lastPose.vecPosition[0]), (pose.vecPosition[1] - lastPose.vecPosition[1]), (pose.vecPosition[2] - lastPose.vecPosition[2]) };
        m_autoDetectSpeeds[i] += (deltas[0] * deltas[0] + deltas[1] * deltas[1] + deltas[2] * deltas[2]) / deltaTime;
    }

    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        double speedI = m_autoDetectSpeeds[i];
        if (isnan(speedI))
            continue;
        if (speedI < k_AUTO_DETECT_MIN_VELOCITY_THRESHOLD)
            continue;

        for (vr::TrackedDeviceIndex_t j = i + 1; j < vr::k_unMaxTrackedDeviceCount; j++) {
            double speedJ = m_autoDetectSpeeds[j];
            if (isnan(speedJ))
                continue;
            if (speedJ < k_AUTO_DETECT_MIN_VELOCITY_THRESHOLD)
                continue;

            double speedDelta = std::abs(speedI - speedJ);
            if (speedDelta < k_AUTO_DETECT_MAX_VELOCITY_DIFF && speedDelta < frameClosestSpeedDelta) {
                frameClosestSpeedDelta = speedDelta;
                frameRefId = i;
                frameTargetId = j;
            }
        }
    }

    // figure out wtf moved
    if (frameRefId != vr::k_unTrackedDeviceIndexInvalid) {
        if (m_candidateRefId == vr::k_unTrackedDeviceIndexInvalid) {
            // found match, start tracking the pair
            m_candidateRefId = frameRefId;
            m_candidateTargetId = frameTargetId;
            m_candidateScore = 1;
        } else if (m_candidateRefId == frameRefId && m_candidateTargetId == frameTargetId) {
            // the same pair we found earlier are moving together
            m_candidateScore++;
        } else {
            // a different pair was found, this may be a false positive so decrease score for now
            m_candidateScore--;
            if (m_candidateScore <= 0) {
                // the new pair is more prominent than the last one so replace it
                m_candidateRefId = frameRefId;
                m_candidateTargetId = frameTargetId;
                m_candidateScore = 1;
            }
        }
    } else {
        // no devices generated a valid pair, did the user stop moving?
        if (m_candidateScore > 0)
            m_candidateScore--;
    }

    if (currentTime - m_autoDetectStartTime >= k_AUTO_DETECT_DURATION) {
        if (m_candidateRefId != vr::k_unTrackedDeviceIndexInvalid && m_candidateTargetId != vr::k_unTrackedDeviceIndexInvalid && m_candidateScore > k_AUTO_DETECT_MINIMUM_SCORE) {
            const auto& refDevInfo = VRState::getInstance()->getVrDevice(m_candidateRefId);
            const auto& targetDevInfo = VRState::getInstance()->getVrDevice(m_candidateTargetId);

            referenceDevice.deviceId = m_candidateRefId;
            referenceDevice.trackingSystem = refDevInfo.szTrackingSystemId;
            referenceDevice.deviceModel = refDevInfo.szModel;
            referenceDevice.deviceSerialNumber = refDevInfo.szSerial;

            targetDevice.deviceId = m_candidateTargetId;
            targetDevice.trackingSystem = targetDevInfo.szTrackingSystemId;
            targetDevice.deviceModel = targetDevInfo.szModel;
            targetDevice.deviceSerialNumber = targetDevInfo.szSerial;

            LOG_CALIB_INFO("Automatically detected devices; ref: ID {}, target: ID {}", m_candidateRefId, m_candidateTargetId);

            // we found a new tracking system pair, update the settings
            CalibrationManager::getInstance()->saveConfig();
            return true;
        } else {
            LOG_CALIB_WARN("Failed to automatically detect devices. Did the user move them enough?");
            return true;
        }
    }
    return false;
}

void TrackingSystemCalibration::resetCalibrationForDevice(const CalibrationDevice& device)
{
    vr::HmdVector3d_t posOrigin = { .v = { 0, 0, 0 } };
    vr::HmdQuaternion_t identityQuat = { .w = 1, .x = 0, .y = 0, .z = 0 };

    ipc::protocol::Command_SetDeviceTransform_t args = {};

    args.unTargetOpenVrDeviceId = device.deviceId;
    args.unRelativeReferenceOpenvrDeviceId = vr::k_unTrackedDeviceIndexInvalid;
    args.unRelativeTargetOpenvrDeviceId = vr::k_unTrackedDeviceIndexInvalid;
    args.enabled(false);
    args.relativeCoordSystem(false);
    args.calibrateMotionVecs(true);
    args.translation = posOrigin;
    args.rotation = identityQuat;
    args.scale = 1.0;

    CalibrationManager::getInstance()->m_ipcClient.SetDeviceTransform(args);
}

// applies calibration to all devices under this tracking system
void TrackingSystemCalibration::apply()
{
    // check if the hmd and tracker are both valid before applying a calibration. if one of them is invalid we shouldnt trust the calibration as its most likely stale anyway
    bool calibrationDevicesAreValid = targetDevice.deviceId < vr::k_unMaxTrackedDeviceCount && referenceDevice.deviceId < vr::k_unMaxTrackedDeviceCount;
    if (referenceDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
        auto refPose = CalibrationManager::getInstance()->m_poses[referenceDevice.deviceId];
        bool actuallyConnected = VRState::getInstance()->getVrDevice(referenceDevice.deviceId).bIsConnected;
        if (!actuallyConnected || !refPose.poseIsValid) {
            calibrationDevicesAreValid = false;
        }
    }

    if (targetDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
        auto targetPose = CalibrationManager::getInstance()->m_poses[targetDevice.deviceId];
        bool actuallyConnected = VRState::getInstance()->getVrDevice(targetDevice.deviceId).bIsConnected;
        if (!actuallyConnected || !targetPose.poseIsValid) {
            calibrationDevicesAreValid = false;
        }
    }

    this->isActive = ((calibrationError == CalibrationError::None) && calibrationDevicesAreValid);

    auto hmdDevice = VRState::getInstance()->getVrDevice(vr::k_unTrackedDeviceIndex_Hmd);
    if (this->hmdIsInReferenceTrackingSystem && hmdDevice.szTrackingSystemId != referenceDevice.trackingSystem) {
        // if the hmd's tracking system is not what was saved, handles users changing streamer / VR headset properly by not applying calibration
        this->isActive = false;
        LOG_CALIB_INFO("Skipping calibration: HMD tracking system changed from {} to {}", referenceDevice.trackingSystem, hmdDevice.szTrackingSystemId);
        return;
    }

    // if we update state the driver will reset calibration nullifying any existing calibrations!
    if (!this->isActive) {
        return;
    }

    LOG_CALIB_INFO("Applying calibration...");
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        auto device = VRState::getInstance()->getVrDevice(i);
        if (device.eDeviceClass != vr::TrackedDeviceClass_Invalid) {
            // if this is not the target tracking system
            if (device.szTrackingSystemId != targetDevice.trackingSystem) {
                continue;
            }

            ipc::protocol::Command_SetDeviceTransform_t args = {};

            args.unTargetOpenVrDeviceId = i;
            args.enabled(true);

            LOG_CALIB_INFO("  applying device [{}]: class: {} connected: {} role: {} tracking: {} model: {} serial: {} idx: {}", i, device.eDeviceClass, device.bIsConnected, device.eControllerRole, device.szTrackingSystemId, device.szModel, device.szSerial, device.dwDeviceIndex);
            args.quirks = ipc::protocol::DeviceQuirks_t::QUIRK_NONE;

            args.translation = eigenAsVrVec3d(calibratedTranslation);
            args.rotation = eigenAsVrQuat(calibratedRotation);
            args.scale = calibratedScale;

            // for relative calibrations
            args.relativeCoordSystem(isContinuousCalibration() && isRelativeCalibration);
            args.calibrateMotionVecs(calibrateMotionVectors);
            args.unRelativeReferenceOpenvrDeviceId = referenceDevice.deviceId;
            args.unRelativeTargetOpenvrDeviceId = targetDevice.deviceId;

            // should only apply to hmd tracker (eg head tracker)
            args.hideContinuousTracker(isContinuousCalibration() && hideContinuousTracker && i == targetDevice.deviceId);
            args.lerpCalibrations(isContinuousCalibration());

            CalibrationManager::getInstance()->m_ipcClient.SetDeviceTransform(args);
        }
    }
}

CalibrationManager::CalibrationManager()
{
    if (s_instance != nullptr) {
        LOG_FATAL("Tried creating CalibrationManager more than once! Breaking singleton. Aborting...");
        return;
    }
    s_instance = this;

    // @FIXME: Need to figure out how to handle multiple calibrations

    // try loading calibrations from disk, if fail, fallback to a default one
    if (loadConfig()) {
        m_needToApplyTransformsAfterInit = true;
    } else {
        TrackingSystemCalibration mainCalibration;
        mainCalibration.hmdIsInReferenceTrackingSystem = true;
        m_calibrations.push_back(mainCalibration);
    }
}

void CalibrationManager::init()
{
    // @TODO:
    if (!m_ipcClient.IsConnected()) {
        m_ipcClient.Connect();

        // initialise memory properly
        m_ipcClient.RequestVirtualDesktopProps();
        m_ipcClient.PollPoses();
    }

    LOG_CALIB_INFO("Initialising CalibrationManager...");
    LOG_CALIB_INFO("Connected OpenVR devices:");
    VRState::getInstance()->debugListDevices();

    // re-use the last calibration now
    if (m_needToApplyTransformsAfterInit) {
        // successfully loaded calibration; update runtime state
        const auto& hmdDevice = VRState::getInstance()->getVrDevice(vr::k_unTrackedDeviceIndex_Hmd);

        for (size_t i = 0; i < m_calibrations.size(); i++) {
            m_calibrations[i].assignTarget(m_calibrations[i].referenceDevice);
            m_calibrations[i].assignTarget(m_calibrations[i].targetDevice);

            // update state
            m_calibrations[i].hmdIsInReferenceTrackingSystem = hmdDevice.szTrackingSystemId == m_calibrations[i].referenceDevice.trackingSystem;
        }
    }

    for (auto& calibration : m_calibrations) {
        calibration.init();
    }

    apply();
}

void CalibrationManager::shutdown()
{
    if (m_ipcClient.IsConnected()) {
        m_ipcClient.Shutdown();
    }
}

void CalibrationManager::start()
{
    // @TODO: maybe more logic here?

    LOG_CALIB_INFO("Starting calibration...");
    LOG_CALIB_INFO("Connected OpenVR devices:");
    VRState::getInstance()->debugListDevices();

    for (auto& calibration : m_calibrations) {
        calibration.start();
    }
}

void CalibrationManager::calibrationTick(const double currentTime)
{
    // try re-connecting to the IPC server (driver) if
    if (!m_ipcClient.IsConnected()) {
        if (currentTime - m_lastIpcConnectionAttemptTime >= k_IPC_CONNECTION_RETRY_INTERVAL_SEC) {
            m_lastIpcConnectionAttemptTime = currentTime;
            m_ipcClient.Connect();

            // initialise memory properly
            m_ipcClient.RequestVirtualDesktopProps();
            m_ipcClient.PollPoses();
            apply();
        }

        // driver is either not ready (we just connected to it) or unavailable (driver is disabled in steamvr addons), its pointless to do calibration logic now so early exit
        return;
    }

    m_ipcClient.RequestVirtualDesktopProps();
    m_ipcClient.PollPoses();

    double wantedInterval = 0;
    size_t countedCalibrations = 0;

    for (auto& calibration : m_calibrations) {
        calibration.calibrationTick(currentTime);
        if (calibration.isActive || calibration.state != CalibrationState::NONE) {
            wantedInterval += calibration.wantedUpdateInterval;
            countedCalibrations++;
        }
    }

    if (countedCalibrations > 0) {
        // @NOTE: is max better here?
        m_wantedUpdateInterval = wantedInterval / countedCalibrations;
    }

    memcpy(m_lastPoses, m_poses, sizeof(m_lastPoses));
    // deviceIsConnected may be stale; keep it in sync
    for (size_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        m_lastPoses[i].deviceIsConnected = VRState::getInstance()->getVrDevice(i).bIsConnected;
    }
}

void CalibrationManager::apply()
{
    if (!m_ipcClient.IsConnected())
        return;

    for (auto& calibration : m_calibrations) {
        calibration.apply();
    }
}

TrackingSystemCalibration& CalibrationManager::getCalibration(const size_t index)
{
    return m_calibrations[index];
}

size_t CalibrationManager::getCalibrationCount() const
{
    return m_calibrations.size();
}

const double CalibrationManager::getWantedUpdateInterval() const
{
    return m_wantedUpdateInterval;
}

bool CalibrationManager::loadConfig()
{
    using namespace spacecal::config::versioned;
    const Configuration* pConfig = spacecal::ConfigurationManager::getInstance()->getConfiguration();

    // no calibrations found
    if (pConfig->calibrations.size() == 0) {
        return false;
    }

    // we need N calibrations
    this->m_calibrations.resize(pConfig->calibrations.size());

    // now load the calibration data
    for (size_t i = 0; i < pConfig->calibrations.size(); i++) {
        this->m_calibrations[i].isActive = pConfig->calibrations[i].is_active;
        // if a calibration was active we assume that the calibration is valid otherwise it wont apply
        this->m_calibrations[i].calibrationError = pConfig->calibrations[i].is_active ? CalibrationError::None : CalibrationError::Unknown;

        // copy device props
        this->m_calibrations[i].targetDevice.deviceModel = pConfig->calibrations[i].target_device.model;
        this->m_calibrations[i].targetDevice.deviceSerialNumber = pConfig->calibrations[i].target_device.serial;
        this->m_calibrations[i].targetDevice.trackingSystem = pConfig->calibrations[i].target_device.tracking_system;

        this->m_calibrations[i].referenceDevice.deviceModel = pConfig->calibrations[i].reference_device.model;
        this->m_calibrations[i].referenceDevice.deviceSerialNumber = pConfig->calibrations[i].reference_device.serial;
        this->m_calibrations[i].referenceDevice.trackingSystem = pConfig->calibrations[i].reference_device.tracking_system;

        this->m_calibrations[i].isRelativeCalibration = pConfig->calibrations[i].anchor_mode == Configuration_Latest::AnchorMode::HmdRelative;
        this->m_calibrations[i].calibrationSpeed = (CalibrationSpeed)pConfig->calibrations[i].calibration_speed;

        // calibrated transform mapping
        // @TODO: should we make a helper func?
        this->m_calibrations[i].calibratedTranslation.x() = pConfig->calibrations[i].calibrated_transform.x;
        this->m_calibrations[i].calibratedTranslation.y() = pConfig->calibrations[i].calibrated_transform.y;
        this->m_calibrations[i].calibratedTranslation.z() = pConfig->calibrations[i].calibrated_transform.z;

        double rollRad = pConfig->calibrations[i].calibrated_transform.roll * (EIGEN_PI / 180.0);
        double pitchRad = pConfig->calibrations[i].calibrated_transform.pitch * (EIGEN_PI / 180.0);
        double yawRad = pConfig->calibrations[i].calibrated_transform.yaw * (EIGEN_PI / 180.0);

        this->m_calibrations[i].calibratedRotation = (Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitchRad, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rollRad, Eigen::Vector3d::UnitX())).normalized();

        this->m_calibrations[i].calibratedScale = pConfig->calibrations[i].scale;

        if (pConfig->calibrations[i].continuous.is_active) {
            this->m_calibrations[i].state = CalibrationState::CONTINUOUS_IDLE;
        }
        this->m_calibrations[i].hideContinuousTracker = pConfig->calibrations[i].continuous.hide_reference_tracker;

        this->m_calibrations[i].calibrateMotionVectors = pConfig->calibrations[i].calibrate_motion_vectors;
        this->m_calibrations[i].autoFixPlayspaceJumps = pConfig->calibrations[i].attempt_auto_fix_playspace_jumps;
        this->m_calibrations[i].enforceMinimumRotationVariance = pConfig->calibrations[i].enforce_minimum_rotation_variance;
    }

    return true;
}

void CalibrationManager::saveConfig() const
{
    using namespace spacecal::config::versioned;
    Configuration* pConfig = spacecal::ConfigurationManager::getInstance()->getConfiguration();

    // we need N calibrations
    pConfig->calibrations.resize(this->m_calibrations.size());

    for (size_t i = 0; i < pConfig->calibrations.size(); i++) {
        pConfig->calibrations[i].is_active = this->m_calibrations[i].isActive;

        // copy device props
        pConfig->calibrations[i].target_device.model = this->m_calibrations[i].targetDevice.deviceModel;
        pConfig->calibrations[i].target_device.serial = this->m_calibrations[i].targetDevice.deviceSerialNumber;
        pConfig->calibrations[i].target_device.tracking_system = this->m_calibrations[i].targetDevice.trackingSystem;

        pConfig->calibrations[i].reference_device.model = this->m_calibrations[i].referenceDevice.deviceModel;
        pConfig->calibrations[i].reference_device.serial = this->m_calibrations[i].referenceDevice.deviceSerialNumber;
        pConfig->calibrations[i].reference_device.tracking_system = this->m_calibrations[i].referenceDevice.trackingSystem;

        pConfig->calibrations[i].calibration_speed = (uint64_t)this->m_calibrations[i].calibrationSpeed;
        // relative RELIES on continuous, so enforce it here
        pConfig->calibrations[i].anchor_mode = (this->m_calibrations[i].isRelativeCalibration && this->m_calibrations[i].isContinuousCalibration()) ? Configuration_Latest::AnchorMode::HmdRelative : Configuration_Latest::AnchorMode::FixedWorld;

        // calibrated transform mapping
        // @TODO: should we make a helper func?
        pConfig->calibrations[i].calibrated_transform.x = (float)this->m_calibrations[i].calibratedTranslation.x();
        pConfig->calibrations[i].calibrated_transform.y = (float)this->m_calibrations[i].calibratedTranslation.y();
        pConfig->calibrations[i].calibrated_transform.z = (float)this->m_calibrations[i].calibratedTranslation.z();

        Eigen::Vector3d euler = this->m_calibrations[i].calibratedRotation.toRotationMatrix().canonicalEulerAngles(2, 1, 0);
        pConfig->calibrations[i].calibrated_transform.yaw = static_cast<float>(euler[0] * 180.0 / EIGEN_PI);
        pConfig->calibrations[i].calibrated_transform.pitch = static_cast<float>(euler[1] * 180.0 / EIGEN_PI);
        pConfig->calibrations[i].calibrated_transform.roll = static_cast<float>(euler[2] * 180.0 / EIGEN_PI);

        pConfig->calibrations[i].scale = (float)this->m_calibrations[i].calibratedScale;

        pConfig->calibrations[i].continuous.is_active = this->m_calibrations[i].isContinuousCalibration();
        pConfig->calibrations[i].continuous.hide_reference_tracker = this->m_calibrations[i].hideContinuousTracker;

        pConfig->calibrations[i].calibrate_motion_vectors = this->m_calibrations[i].calibrateMotionVectors;
        pConfig->calibrations[i].attempt_auto_fix_playspace_jumps = this->m_calibrations[i].autoFixPlayspaceJumps;
        pConfig->calibrations[i].enforce_minimum_rotation_variance = this->m_calibrations[i].enforceMinimumRotationVariance;
    }

    ConfigurationError err = spacecal::ConfigurationManager::getInstance()->saveConfiguration();
    if (err != ConfigurationError::Ok) {
        LOG_WARNING("Failed to write calibration, got {}", (uint32_t)err);
    }
}
}