/**
* Patch defense runtime utilities for ORB-SLAM2.
*
* @author Korey
* @Date 2025-01-05
*/

#ifndef PATCH_DEFENSE_H
#define PATCH_DEFENSE_H

#include <opencv2/core/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace ORB_SLAM2
{

class Frame;

struct MotionGuardConfig
{
    // Tracking 级冻结守卫：背景静止 + 可疑区域运动 -> 冻结位姿并可选禁止关键帧
    bool enable_motion_guard = false;
    // 0=auto（优先基于权重分组，不足则退化为全局分布）；1=weights_only；2=global_only
    int source = 0;
    // 0=仅冻结位姿；1=冻结位姿并禁止关键帧
    int mode = 1;
    // 防止初始化早期误触发导致 reset：只有当地图关键帧数量达到该值才允许冻结
    int min_keyframes = 5;
    float safe_weight_threshold = 0.6f;      // >= 该阈值视为“安全背景点”
    float suspect_weight_threshold = 0.4f;   // <= 该阈值视为“可疑点”
    int min_safe_points = 80;
    int min_suspect_points = 30;
    float static_px = 0.5f;                 // 背景静止阈值（中位光流，px）
    float patch_px = 1.5f;                  // 可疑点运动阈值（中位光流，px）
    float mismatch_px = 1.0f;               // 可疑-安全的运动差阈值（px）
    float moving_ratio_threshold = 0.35f;   // 可疑点中 >patch_px 的比例阈值
    int hold_on_frames = 2;                 // 连续触发 N 帧后开启 guard
    int hold_off_frames = 4;                // 连续不触发 N 帧后关闭 guard
    int win_size = 21;                      // LK 光流窗口
    int pyr_levels = 3;                     // LK 金字塔层数
    bool log_motion_guard = false;
};

struct PatchDefenseConfig
{
    bool use_patch_defense = false;
    bool enable_risk_mask = true;
    // Risk mask evaluation scope:
    // 0 = global (compute on full image)
    // 1 = local_roi (compute only around candidates)
    int risk_mask_scope = 0;
    bool enable_local_statistics = true;
    bool enable_density_stat = true;
    bool enable_descriptor_stat = true;
    bool enable_orientation_stat = true;
    // Candidate gating (stage-1): weighted score threshold.
    // score = sum(w_k * r_k) / sum(w_k), k in enabled {density, descriptor, orientation}
    // candidate if score >= candidate_score_threshold.
    float candidate_score_threshold = 0.25f;
    float candidate_weight_density = 1.0f;
    float candidate_weight_descriptor = 1.0f;
    float candidate_weight_orientation = 1.0f;
    bool enable_temporal_consistency = true;
    // Temporal consistency scope:
    // 0 = global (apply to all keypoints)
    // 1 = candidate (apply only to candidates)
    int temporal_scope = 0;
    bool enable_weighted_frontend = true;
    bool enable_weighted_ba = true;
    bool enable_loop_defense = true;
    MotionGuardConfig motion_guard;
    bool enable_pyramid_risk = true;
    int grid_size = 16;
    int entropy_bins = 32;
    int context_radius = 2;
    float lambda_context = 1.3f;
    float alpha_texture = 0.85f;
    float alpha_orientation = 0.9f;
    int orientation_bins = 16;
    float orientation_threshold = 2.7f;
    int gaussian_kernel = 5;
    float gaussian_sigma = 1.0f;
    int dilation_iterations = 1;
    std::vector<int> multi_scale_cells;
    float density_quantile = 0.7f;
    float descriptor_quantile = 0.2f;
    float descriptor_high_quantile = 0.95f;
    float orientation_quantile = 0.8f;
    int desc_min_points = 2;
    int orient_min_points = 2;
    float ema_alpha_density = 0.8f;
    float ema_alpha_descriptor = 0.8f;
    float burst_lambda_density = 1.0f;
    float burst_lambda_descriptor = 1.0f;
    float eta_temporal = 0.6f;
    float w_min = 0.08f;
    float w_floor = 0.001f;
    float safe_weight_threshold = 0.5f;
    float tau_safe = 0.3f;
    float loop_cover_reject = 0.05f;      // 覆盖率低于此直接拒绝
    float loop_cover_no_kl = 0.20f;       // 覆盖率低于此跳过 KL，仅看 safeRatio
    float loop_cover_loose = 0.35f;       // 覆盖率低于此使用宽松 KL
    float loop_kl_loose = 1.0f;           // 宽松 KL 阈值
    float loop_kl_strict = 0.4f;          // 严格 KL 阈值
    int histogram_bins = 8;
    float histogram_epsilon = 1.0e-4f;
    // 回环开关积分器：当 safeRatio 极高时允许一次性“快速放行”
    float loop_safe_ratio_boost = 0.9f;    // safeRatio >= 该值时使用 boost 增量；<=tau_safe 视为关闭
    float loop_delta_pos_boost = 1.0f;     // boost 增量（建议 >= loop_state_on 以实现“一次验证即放行”）
    float loop_delta_pos = 0.6f;
    float loop_delta_neg = 0.3f;
    float loop_state_on = 0.65f;
    float loop_state_off = 0.3f;
    // 权重缩放系数
    float k_mask_weight = 1.0f;
    float k_temp_weight = 1.0f;
    float k_risk_mask = 1.0f;
    bool logging = true;
    bool log_risk_mask = true;
    bool log_density = true;
    bool log_descriptor = true;
    bool log_orientation = true;
    bool log_candidate_gate = false;
    bool log_temporal = false;
    bool log_loop_defense = true;
    // 权重融合系数（归一后作为指数，用于偏向某分支）
    float weight_mask = 1.0f;
    float weight_temp = 1.0f;
    // 关键帧安全判定
    bool keyframe_use_safe_ratio = false;
    float keyframe_safe_ratio_min = 0.5f;

    // 回环匹配可视化输出（与 use_patch_defense 无关；只要配置了路径就会输出）
    std::string loop_match_output_dir;
};

bool LoadPatchDefenseConfig(PatchDefenseConfig &config);
void SetPatchDefenseConfig(const PatchDefenseConfig &config);
const PatchDefenseConfig& GetPatchDefenseConfig();

class PatchDefense
{
public:
    PatchDefense();

    void SetConfig(const PatchDefenseConfig &config);
    void Configure(const PatchDefenseConfig &config, const cv::Size &imageSize, int nLevels, const std::vector<float> &scaleFactors);

    void ProcessFrame(Frame &frame, const cv::Mat &gray);

    bool GetRiskMask(cv::Mat &out) const;

    float GetMeanRisk() const { return mLastMeanRisk; }
    float GetSafeRatio() const { return mLastSafeRatio; }

    bool IsEnabled() const { return mbEnabled; }

private:
    void ComputeRiskMaps(const cv::Mat &gray, const std::vector<uint8_t> *candidates, const Frame *frame);
    void AssignRiskToKeypoints(Frame &frame, const std::vector<uint8_t> *candidates);
    void ComputeLocalStatistics(Frame &frame);
    void ComputeTemporalWeights(Frame &frame);
    void CombineWeights(Frame &frame);

    float SampleRiskAt(const Frame &frame, size_t idx, int octave) const;
    void EnsureTemporalBuffers();

    struct GridTemporalState
    {
        float mu_density = 0.f;
        float var_density = 0.f;
        float mu_descriptor = 0.f;
        float var_descriptor = 0.f;
        bool initialized = false;
    };

    PatchDefenseConfig mConfig;
    bool mbConfigured = false;
    bool mbEnabled = false;

    cv::Size mBaseSize;
    int mnLevels = 0;
    std::vector<float> mScaleFactors;
    std::vector<float> mInvScaleFactors;

    int mGridCols = 0;
    int mGridRows = 0;

    std::vector<cv::Mat> mRiskPyramid;
    cv::Mat mLastRiskMask;

    std::vector<std::vector<std::vector<GridTemporalState> > > mTemporalStates;

    float mLastMeanRisk = 0.f;
    float mLastSafeRatio = 0.f;
    float mLastDensityMin = 0.f;
    float mLastDensityMax = 0.f;
    float mLastDensityMean = 0.f;
    int mLastDensityHighCount = 0;
    float mLastDescriptorMin = 0.f;
    float mLastDescriptorMax = 0.f;
    float mLastDescriptorMean = 0.f;
    int mLastDescriptorHighCount = 0;
    float mLastOrientationMin = 0.f;
    float mLastOrientationMax = 0.f;
    float mLastOrientationMean = 0.f;
    int mLastOrientationHighCount = 0;
    int mLastSafeCount = 0;
    int mLastPointCount = 0;
    std::vector<std::string> mLastDensityLogs;
    std::vector<std::string> mLastDescriptorLogs;
    std::vector<std::string> mLastOrientationLogs;

    std::vector<float> mDensityRisks;
    std::vector<float> mDescriptorRisks;
    std::vector<float> mOrientationRisks;
    std::vector<float> mTemporalRaw;
    std::vector<float> mTemporalWeights;
};

} // namespace ORB_SLAM2

#endif // PATCH_DEFENSE_H
