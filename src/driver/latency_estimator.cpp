#include "latency_estimator.h"
#include <algorithm>
#include <chrono>
#include <float.h>

namespace spacecal {
void DeviceVelocityHistory::push_sample(double time, const Eigen::Vector3d& position)
{
    // if prevTime == 0 we dont have a prior sample
    if (m_prevTime == 0.0) {
        m_prevTime = time;
        m_prevPos = position;
        return;
    }

    double deltaTime = time - m_prevTime;
    if (deltaTime > 1e-6) {
        Eigen::Vector3d velocity = (position - m_prevPos) / deltaTime;

        m_samples[m_head] = {
            .time = 0.5 * (m_prevTime + time), // normalise possible different tracking frequencies
            .speed = velocity.norm(),
        };

        m_head = (m_head + 1) % k_MAX_DEVICE_SAMPLES;
        m_count = std::min(m_count + 1, k_MAX_DEVICE_SAMPLES);
        ++m_frameCounter;
    }
    m_prevTime = time;
    m_prevPos = position;
}

const DeviceVelocityHistory::LatencySample_t& DeviceVelocityHistory::at(size_t i) const
{
    size_t oldestPhysIdx = (m_count < k_MAX_DEVICE_SAMPLES) ? 0 : m_head;
    size_t physIdx = (oldestPhysIdx + i) % k_MAX_DEVICE_SAMPLES;
    return m_samples[physIdx];
}

bool DeviceVelocityHistory::speed_at(double time, double& outSpeed) const
{
    double fOldestTime = m_count == 0 ? 0.0 : this->at(0).time;
    double fNewestTime = m_count == 0 ? 0.0 : this->at(m_count - 1).time;
    if (m_count < 2 || time < fOldestTime || time > fNewestTime)
        return false;

    size_t lowerIdx = 0;
    size_t upperIdx = m_count - 1;
    while (upperIdx - lowerIdx > 1) {
        size_t midIdx = lowerIdx + (upperIdx - lowerIdx) / 2;
        if (this->at(midIdx).time <= time) {
            lowerIdx = midIdx;
        } else {
            upperIdx = midIdx;
        }
    }

    const LatencySample_t& earlierSample = this->at(lowerIdx);
    const LatencySample_t& laterSample = this->at(upperIdx);
    double deltaTime = laterSample.time - earlierSample.time;
    double lerpFactor = (deltaTime > 1e-9) ? (time - earlierSample.time) / deltaTime : 0.0;
    outSpeed = earlierSample.speed + lerpFactor * (laterSample.speed - earlierSample.speed);
    return true;
}

inline Eigen::Quaterniond eigenFromHmdQuat(const vr::HmdQuaternion_t& quat)
{
    return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}
inline Eigen::Vector3d eigenVecFromHmdVec(const double pos[3])
{
    return Eigen::Vector3d(pos[0], pos[1], pos[2]);
}

inline Eigen::Vector3d getWorldFromDriverPositionRaw(const vr::DriverPose_t& pose)
{
    Eigen::Quaterniond worldFromDriverRot = eigenFromHmdQuat(pose.qWorldFromDriverRotation);
    Eigen::Vector3d worldFromDriverPos = eigenVecFromHmdVec(pose.vecWorldFromDriverTranslation);
    return worldFromDriverPos + worldFromDriverRot * eigenVecFromHmdVec(pose.vecPosition);
}

void LatencyEstimator::push_pose(vr::TrackedDeviceIndex_t deviceIndex, const vr::DriverPose_t& driverPose)
{
    if (deviceIndex >= vr::k_unMaxTrackedDeviceCount)
        return;

    if (!driverPose.deviceIsConnected || !driverPose.poseIsValid || driverPose.result != vr::ETrackingResult::TrackingResult_Running_OK)
        return;

    // @NOTE: poseTimeOffset is intentionally IGNORED. most drivers are poorly written and LIE about their
    //        velocity AND latency which is the motivation for this entire latency estimator's existence!
    double nowSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    m_histories[deviceIndex].push_sample(nowSeconds, getWorldFromDriverPositionRaw(driverPose));
}

double LatencyEstimator::weighted_mean_of(const std::vector<double>& values, const std::vector<double>& weights, double totalWeight) const
{
    double weightedSum = 0.0;
    for (size_t i = 0; i < values.size(); ++i) {
        weightedSum += weights[i] * values[i];
    }
    return weightedSum / totalWeight;
}

double LatencyEstimator::weighted_standard_deviation_of(const std::vector<double>& values, const std::vector<double>& weights, double totalWeight, double mean) const
{
    double weightedSumSquaredDeviation = 0.0;
    for (size_t i = 0; i < values.size(); ++i) {
        double deviation = values[i] - mean;
        weightedSumSquaredDeviation += weights[i] * deviation * deviation;
    }
    return std::sqrt(weightedSumSquaredDeviation / totalWeight);
}

// DBL_MAX if invalid
LatencyEstimator::CurveDifferenceResult_t LatencyEstimator::curve_difference_at(const DeviceVelocityHistory& ref, const DeviceVelocityHistory& target, double deltaTime) const
{
    // compute a weighted score for how well the two curves align at the given deltaTime
    // weight is to minimise the effects of low velocity
    std::vector<double> pairedRefSpeeds;
    std::vector<double> pairedTargetSpeeds;
    std::vector<double> pairedWeights;
    pairedRefSpeeds.reserve(ref.count());
    pairedTargetSpeeds.reserve(ref.count());
    pairedWeights.reserve(ref.count());

    double totalWeight = 0.0;

    for (size_t i = 0; i < ref.count(); ++i) {
        const auto& refSample = ref.at(i);
        double targetSpeed;
        if (!target.speed_at(refSample.time - deltaTime, targetSpeed))
            continue;

        double weight = std::min(refSample.speed, targetSpeed);

        pairedRefSpeeds.push_back(refSample.speed);
        pairedTargetSpeeds.push_back(targetSpeed);
        pairedWeights.push_back(weight);
        totalWeight += weight;
    }

    size_t sampleCount = pairedRefSpeeds.size();
    if (sampleCount < k_MIN_PAIRED_SAMPLES) {
        return {
            .difference = DBL_MAX,
            .totalWeight = totalWeight,
            .sampleCount = sampleCount,
        };
    }

    if (totalWeight < k_MIN_USEFUL_WEIGHT) {
        return {
            .difference = DBL_MAX,
            .totalWeight = totalWeight,
            .sampleCount = sampleCount,
        };
    }

    double refMean = weighted_mean_of(pairedRefSpeeds, pairedWeights, totalWeight);
    double targetMean = weighted_mean_of(pairedTargetSpeeds, pairedWeights, totalWeight);
    double refSpread = weighted_standard_deviation_of(pairedRefSpeeds, pairedWeights, totalWeight, refMean);
    double targetSpread = weighted_standard_deviation_of(pairedTargetSpeeds, pairedWeights, totalWeight, targetMean);

    constexpr double kMinSpread = 1e-6;
    if (refSpread < kMinSpread || targetSpread < kMinSpread) {
        return {
            .difference = DBL_MAX,
            .totalWeight = totalWeight,
            .sampleCount = sampleCount,
        };
    }

    double weightedSumSquaredDifference = 0.0;
    for (size_t i = 0; i < sampleCount; ++i) {
        double normalizedRefSpeed = (pairedRefSpeeds[i] - refMean) / refSpread;
        double normalizedTargetSpeed = (pairedTargetSpeeds[i] - targetMean) / targetSpread;
        double difference = normalizedRefSpeed - normalizedTargetSpeed;
        weightedSumSquaredDifference += pairedWeights[i] * difference * difference;
    }

    return {
        .difference = weightedSumSquaredDifference / totalWeight,
        .totalWeight = totalWeight,
        .sampleCount = sampleCount,
    };
}

// DBL_MAX if invalid
double LatencyEstimator::regularized_difference_at(const DeviceVelocityHistory& ref, const DeviceVelocityHistory& target, double deltaTime, double priorLatency) const
{
    double difference = curve_difference_at(ref, target, deltaTime).difference;
    if (difference == DBL_MAX)
        return DBL_MAX;

    double distanceFromPrior = deltaTime - priorLatency;
    return difference + k_CONTINUITY_BIAS_WEIGHT * (distanceFromPrior * distanceFromPrior);
}

// DBL_MAX if invalid
double LatencyEstimator::compute_latency(const DeviceVelocityHistory& ref, const DeviceVelocityHistory& target, double priorLatency, double& outConfidence) const
{
    if (ref.count() < k_MIN_PAIRED_SAMPLES || target.count() < k_MIN_PAIRED_SAMPLES)
        return DBL_MAX;

    double bestDeltaTime = 0.0;
    double bestDifference = DBL_MAX;
    double secondBestDifference = DBL_MAX;

    // first minimise search space
    for (int i = 0; i < k_NUM_COARSE_SEARCH_STEPS; ++i) {
        double candidateDelaTime = -k_MAX_LATENCY_SECONDS + (2.0 * k_MAX_LATENCY_SECONDS) * (double(i) / (k_NUM_COARSE_SEARCH_STEPS - 1));
        double difference = regularized_difference_at(ref, target, candidateDelaTime, priorLatency);
        if (difference < bestDifference) {
            if (std::abs(bestDeltaTime - candidateDelaTime) > k_MIN_PEAK_SEPARATION_SECONDS && bestDifference < secondBestDifference) {
                secondBestDifference = bestDifference;
            }
            bestDifference = difference;
            bestDeltaTime = candidateDelaTime;
        } else if (difference < secondBestDifference && std::abs(candidateDelaTime - bestDeltaTime) > k_MIN_PEAK_SEPARATION_SECONDS) {
            secondBestDifference = difference;
        }
    }

    if (bestDifference > k_MAX_ACCEPTABLE_DIFFERENCE)
        return DBL_MAX;

    // golden-section search to refine the result closer towards ground truth
    double step = (2.0 * k_MAX_LATENCY_SECONDS) / (k_NUM_COARSE_SEARCH_STEPS - 1);
    double lo = bestDeltaTime - step;
    double hi = bestDeltaTime + step;
    constexpr double k_GOLDEN_RATIO_MINUS_ONE = 0.6180339887498949;

    double x1 = hi - k_GOLDEN_RATIO_MINUS_ONE * (hi - lo);
    double x2 = lo + k_GOLDEN_RATIO_MINUS_ONE * (hi - lo);
    double f1 = regularized_difference_at(ref, target, x1, priorLatency);
    double f2 = regularized_difference_at(ref, target, x2, priorLatency);

    for (int iter = 0; iter < 24; ++iter) {
        if (f1 < f2) {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - k_GOLDEN_RATIO_MINUS_ONE * (hi - lo);
            f1 = regularized_difference_at(ref, target, x1, priorLatency);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + k_GOLDEN_RATIO_MINUS_ONE * (hi - lo);
            f2 = regularized_difference_at(ref, target, x2, priorLatency);
        }
    }

    double finalDeltaTime = 0.5 * (lo + hi);

    LatencyEstimator::CurveDifferenceResult_t finalResult = curve_difference_at(ref, target, finalDeltaTime);
    if (finalResult.difference > k_MAX_ACCEPTABLE_DIFFERENCE)
        return DBL_MAX;

    double ambiguityRatio = (secondBestDifference == DBL_MAX)
        ? INFINITY : secondBestDifference / std::max(bestDifference, 1e-9);

    // range 0...1
    outConfidence = confidence_from_difference(finalResult.difference)
        * confidence_from_ambiguity(ambiguityRatio)
        * confidence_from_sample_support(finalResult.totalWeight, finalResult.sampleCount);

    return finalDeltaTime;
}

double LatencyEstimator::confidence_from_difference(double difference) const
{
    if (difference == DBL_MAX) {
        return 0.0;
    }
    return std::clamp(1.0 - (difference / k_MAX_ACCEPTABLE_DIFFERENCE), 0.0, 1.0);
}

double LatencyEstimator::confidence_from_ambiguity(double ambiguityRatio) const
{
    if (!std::isfinite(ambiguityRatio)) {
        return 1.0;
    }

    double t = (ambiguityRatio - k_AMBIGUITY_RATIO_LOW) / (k_AMBIGUITY_RATIO_HIGH - k_AMBIGUITY_RATIO_LOW);
    return std::clamp(t, 0.0, 1.0);
}

double LatencyEstimator::confidence_from_sample_support(double totalWeight, size_t sampleCount) const
{
    double weightConfidence = std::clamp((totalWeight - k_MIN_USEFUL_WEIGHT) / (k_MIN_USEFUL_WEIGHT * k_SAMPLE_WEIGHT_SUPPORT_SCALE), 0.0, 1.0);
    double countConfidence = std::clamp((double(sampleCount) - double(k_MIN_PAIRED_SAMPLES)) / (double(k_MIN_PAIRED_SAMPLES) * k_SAMPLE_COUNT_SUPPORT_SCALE), 0.0, 1.0);
    return std::min(weightConfidence, countConfidence);
}

double LatencyEstimator::get_latency(vr::TrackedDeviceIndex_t refIndex, vr::TrackedDeviceIndex_t targetIndex, double& outConfidence)
{
    if (refIndex >= vr::k_unMaxTrackedDeviceCount || targetIndex >= vr::k_unMaxTrackedDeviceCount || refIndex == targetIndex) {
        outConfidence = 0.0;
        return 0.0;
    }

    CacheEntry_t& entry = m_cache[refIndex][targetIndex];
    const DeviceVelocityHistory& ref = m_histories[refIndex];
    const DeviceVelocityHistory& target = m_histories[targetIndex];

    uint64_t refFrames = ref.frame_counter();
    uint64_t targetFrames = target.frame_counter();
    if (entry.referenceFrameCount.load(std::memory_order_acquire) == refFrames && entry.targetFrameCount.load(std::memory_order_acquire) == targetFrames) {
        outConfidence = entry.confidence.load(std::memory_order_acquire);
        return entry.latency.load(std::memory_order_acquire);
    }

    bool expected = false;
    if (!entry.claimed.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        outConfidence = entry.confidence.load(std::memory_order_acquire);
        return entry.latency.load(std::memory_order_acquire);
    }

    double timeNowSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double current = entry.latency.load(std::memory_order_acquire);
    double confidence = entry.confidence.load(std::memory_order_acquire);
    double rawConfidence = 0.0;
    double rawLatency = compute_latency(ref, target, current, rawConfidence);

    entry.referenceFrameCount.store(refFrames, std::memory_order_relaxed);
    entry.targetFrameCount.store(targetFrames, std::memory_order_relaxed);

    if (rawLatency == DBL_MAX) {
        outConfidence = entry.confidence.load(std::memory_order_relaxed) * k_CONFIDENCE_DECAY_ON_MISS;
        entry.confidence.store(outConfidence, std::memory_order_release);
        entry.lastTimeSeconds = timeNowSeconds;
        entry.claimed.store(false, std::memory_order_release);
        return current;
    }

    double smoothedConfidence = entry.confidenceSmoother.filter(rawConfidence, timeNowSeconds);
    entry.confidence.store(std::clamp(smoothedConfidence, 0.0, 1.0), std::memory_order_release);

    bool hadEstimate = entry.hasEstimate.load(std::memory_order_relaxed);

    if (!hadEstimate) {
        double smoothed = entry.smoother.filter(rawLatency, timeNowSeconds);
        entry.latency.store(smoothed, std::memory_order_release);
        entry.hasEstimate.store(true, std::memory_order_relaxed);
    } else {
        double differenceAtCurrent = regularized_difference_at(ref, target, current, current);
        double differenceAtCandidate = regularized_difference_at(ref, target, rawLatency, current);

        bool candidateIsMeaningfullyBetter = (differenceAtCurrent == DBL_MAX) || (differenceAtCandidate < differenceAtCurrent - k_MIN_IMPROVEMENT_TO_SWITCH);

        if (candidateIsMeaningfullyBetter) {
            double smoothed = entry.smoother.filter(rawLatency, timeNowSeconds);
            entry.latency.store(smoothed, std::memory_order_release);
        }
    }

    entry.lastTimeSeconds = timeNowSeconds;
    entry.claimed.store(false, std::memory_order_release);
    outConfidence = entry.confidence.load(std::memory_order_acquire);
    return entry.latency.load(std::memory_order_acquire);
}
}
