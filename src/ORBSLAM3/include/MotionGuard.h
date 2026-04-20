/**
 * MotionGuard.h
 * 基于“背景静止 + 可疑区域运动”的 Tracking 级冻结守卫。
 *
 * @author Korey
 * @Date 2025-01-05
 */

#ifndef ORB_SLAM3_MOTION_GUARD_H
#define ORB_SLAM3_MOTION_GUARD_H

#include <opencv2/core/core.hpp>
#include <vector>

#include "Frame.h"
#include "PatchDefense.h"

namespace ORB_SLAM3
{

struct MotionGuardStats
{
    bool active = false;
    bool trigger = false;
    int tracked_total = 0;
    int tracked_safe = 0;
    int tracked_suspect = 0;
    float median_flow_safe_px = 0.f;
    float median_flow_suspect_px = 0.f;
    float mismatch_px = 0.f;
    float moving_ratio_suspect = 0.f;
    float q10_flow_px = 0.f;
    float q50_flow_px = 0.f;
    float q90_flow_px = 0.f;
    float moving_ratio_total = 0.f;
};

class MotionGuard
{
public:
    MotionGuard() = default;

    void SetConfig(const MotionGuardConfig &config);
    void Reset();

    // 基于上一帧图像与上一帧关键点（含 mvPatchWeights）评估当前帧是否进入冻结守卫。
    // 返回：当前是否处于 guard active 状态（带滞回）。
    bool Update(const cv::Mat &prevGray, const Frame &prevFrame, const cv::Mat &curGray);

    bool IsActive() const { return mbActive; }
    const MotionGuardStats& GetLastStats() const { return mLast; }

private:
    float Median(std::vector<float> &values) const;

    MotionGuardConfig mConfig;
    bool mbActive = false;
    int mOnCount = 0;
    int mOffCount = 0;
    MotionGuardStats mLast;
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_MOTION_GUARD_H
