/**
 * MotionGuard.cc
 * Detects a static background plus suspicious motion using 2D optical flow, and provides freeze-guard decisions at the Tracking layer.
 *
 * @author Korey
 * @Date 2025-01-05
 */

#include "MotionGuard.h"

#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>

namespace ORB_SLAM3
{

namespace
{

template<typename T>
T Clamp(const T &v, const T &lo, const T &hi)
{
    return std::min(std::max(v, lo), hi);
}

} // namespace

void MotionGuard::SetConfig(const MotionGuardConfig &config)
{
    mConfig = config;
    Reset();
}

void MotionGuard::Reset()
{
    mbActive = false;
    mOnCount = 0;
    mOffCount = 0;
    mLast = MotionGuardStats();
}

float MotionGuard::Median(std::vector<float> &values) const
{
    if(values.empty())
        return 0.f;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    return values[mid];
}

namespace
{

float Quantile(std::vector<float> &values, float q)
{
    if(values.empty())
        return 0.f;
    q = Clamp(q, 0.f, 1.f);
    const size_t idx = static_cast<size_t>(std::round(q * (values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + idx, values.end());
    return values[idx];
}

} // namespace

bool MotionGuard::Update(const cv::Mat &prevGray, const Frame &prevFrame, const cv::Mat &curGray)
{
    mLast = MotionGuardStats();

    if(!mConfig.enable_motion_guard)
    {
        mbActive = false;
        return false;
    }

    if(prevGray.empty() || curGray.empty() || prevGray.size() != curGray.size())
        return mbActive;

    if(prevFrame.N <= 0)
        return mbActive;

    const float wSafe = Clamp(mConfig.safe_weight_threshold, 0.f, 1.f);
    const float wSuspect = Clamp(mConfig.suspect_weight_threshold, 0.f, 1.f);
    const float staticPx = std::max(0.f, mConfig.static_px);
    const float patchPx = std::max(0.f, mConfig.patch_px);
    const float mismatchPx = std::max(0.f, mConfig.mismatch_px);
    const float movingRatioThr = Clamp(mConfig.moving_ratio_threshold, 0.f, 1.f);

    const int win = std::max(5, mConfig.win_size);
    const int winOdd = (win % 2 == 0) ? win + 1 : win;
    const int levels = std::max(0, mConfig.pyr_levels);

    // First run the static-background plus moving-subset detector on all optical-flow tracks; this does not depend on weights, so it still works when the patch is not down-weighted.
    std::vector<cv::Point2f> ptsPrevAll;
    ptsPrevAll.reserve(prevFrame.N);
    std::vector<uint8_t> label;
    label.reserve(prevFrame.N);
    for(int i = 0; i < prevFrame.N; ++i)
    {
        ptsPrevAll.push_back(prevFrame.mvKeys[i].pt);
        const float w = prevFrame.GetKeypointWeight(static_cast<size_t>(i));
        if(w >= wSafe)
            label.push_back(1);
        else if(w <= wSuspect)
            label.push_back(2);
        else
            label.push_back(0);
    }

    if(ptsPrevAll.empty())
        return mbActive;

    std::vector<cv::Point2f> ptsCurAll;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prevGray, curGray, ptsPrevAll, ptsCurAll, status, err, cv::Size(winOdd, winOdd), levels);

    std::vector<float> flowAll;
    flowAll.reserve(ptsPrevAll.size());
    int trackedTotal = 0;
    int movingTotal = 0;
    for(size_t k = 0; k < ptsPrevAll.size(); ++k)
    {
        if(k >= status.size() || !status[k])
            continue;
        const float mag = static_cast<float>(cv::norm(ptsCurAll[k] - ptsPrevAll[k]));
        flowAll.push_back(mag);
        trackedTotal++;
        if(mag > patchPx)
            movingTotal++;
    }

    mLast.tracked_total = trackedTotal;
    mLast.moving_ratio_total = (trackedTotal > 0) ? static_cast<float>(movingTotal) / trackedTotal : 0.f;

    float q10 = 0.f, q50 = 0.f, q90 = 0.f;
    if(!flowAll.empty())
    {
        std::vector<float> tmp = flowAll;
        q10 = Quantile(tmp, 0.10f);
        tmp = flowAll;
        q50 = Quantile(tmp, 0.50f);
        tmp = flowAll;
        q90 = Quantile(tmp, 0.90f);
    }
    mLast.q10_flow_px = q10;
    mLast.q50_flow_px = q50;
    mLast.q90_flow_px = q90;

    const bool enoughTotal = trackedTotal >= std::max(0, mConfig.min_safe_points);
    const bool backgroundStaticGlobal = enoughTotal && (q10 <= staticPx);
    const bool movingSubsetGlobal = enoughTotal &&
        (q90 >= patchPx) &&
        ((q90 - q10) >= mismatchPx) &&
        (mLast.moving_ratio_total >= movingRatioThr);
    const bool triggerGlobal = backgroundStaticGlobal && movingSubsetGlobal;

    // Then try the weight-grouped detector, which is more conservative about false positives, but can fail when the weights do not suppress the patch.
    bool triggerWeights = false;
    int safeTracked = 0;
    int suspectTracked = 0;
    int suspectMoving = 0;
    float medSafe = 0.f;
    float medSus = 0.f;
    if(mConfig.source != 2)
    {
        std::vector<float> flowSafe;
        std::vector<float> flowSuspect;
        flowSafe.reserve(ptsPrevAll.size());
        flowSuspect.reserve(ptsPrevAll.size());

        for(size_t i = 0; i < ptsPrevAll.size(); ++i)
        {
            if(i >= status.size() || !status[i])
                continue;
            if(i >= label.size())
                continue;
            const uint8_t cls = label[i];
            if(cls == 0)
                continue;
            const float mag = static_cast<float>(cv::norm(ptsCurAll[i] - ptsPrevAll[i]));
            if(cls == 1)
            {
                flowSafe.push_back(mag);
                safeTracked++;
            }
            else if(cls == 2)
            {
                flowSuspect.push_back(mag);
                suspectTracked++;
                if(mag > patchPx)
                    suspectMoving++;
            }
        }

        medSafe = Median(flowSafe);
        medSus = Median(flowSuspect);
        mLast.tracked_safe = safeTracked;
        mLast.tracked_suspect = suspectTracked;
        mLast.median_flow_safe_px = medSafe;
        mLast.median_flow_suspect_px = medSus;
        mLast.mismatch_px = medSus - medSafe;
        mLast.moving_ratio_suspect = (suspectTracked > 0) ? static_cast<float>(suspectMoving) / suspectTracked : 0.f;

        const bool enoughSafe = safeTracked >= std::max(0, mConfig.min_safe_points);
        const bool enoughSus = suspectTracked >= std::max(0, mConfig.min_suspect_points);
        const bool backgroundStatic = enoughSafe && (medSafe <= staticPx);
        const bool suspectMovingOk = enoughSus &&
            (medSus >= patchPx) &&
            ((medSus - medSafe) >= mismatchPx) &&
            (mLast.moving_ratio_suspect >= movingRatioThr);
        triggerWeights = backgroundStatic && suspectMovingOk;
    }

    bool triggerNow = false;
    if(mConfig.source == 1)
        triggerNow = triggerWeights;
    else if(mConfig.source == 2)
        triggerNow = triggerGlobal;
    else
        triggerNow = triggerWeights || triggerGlobal;

    mLast.trigger = triggerNow;

    if(!mbActive)
    {
        if(triggerNow)
            mOnCount++;
        else
            mOnCount = 0;

        if(mOnCount >= std::max(1, mConfig.hold_on_frames))
        {
            mbActive = true;
            mOffCount = 0;
        }
    }
    else
    {
        if(!triggerNow)
            mOffCount++;
        else
            mOffCount = 0;

        if(mOffCount >= std::max(1, mConfig.hold_off_frames))
        {
            mbActive = false;
            mOnCount = 0;
        }
    }

    mLast.active = mbActive;
    return mbActive;
}

} // namespace ORB_SLAM3
