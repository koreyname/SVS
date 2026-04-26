/**
* Patch defense runtime utilities for ORB-SLAM3.
*
* @author Korey
* @Date 2025-01-05
*/

#ifndef ORB_SLAM3_PATCH_DEFENSE_H
#define ORB_SLAM3_PATCH_DEFENSE_H

#include <opencv2/core/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace ORB_SLAM3
{

class Frame;

struct MotionGuardConfig
{
    // Tracking-level freeze guard: static background plus suspicious-region motion -> freeze the pose and optionally block keyframes.
    bool enable_motion_guard = false;
    // 0=auto (prefer weight-based grouping, fall back to global distribution when insufficient); 1=weights_only; 2=global_only
    int source = 0;
    // 0=freeze pose only; 1=freeze pose and block keyframes.
    int mode = 1;
    // Prevent early-initialization false triggers from causing resets: allow freezing only after the map reaches this number of keyframes.
    int min_keyframes = 5;
    float safe_weight_threshold = 0.6f;      // >= this threshold is treated as a safe background point
    float suspect_weight_threshold = 0.4f;   // <= this threshold is treated as a suspicious point
    int min_safe_points = 80;
    int min_suspect_points = 30;
    float static_px = 0.5f;                 // Static-background threshold (median optical flow, px)
    float patch_px = 1.5f;                  // Suspicious-point motion threshold (median optical flow, px)
    float mismatch_px = 1.0f;               // Suspicious-safe motion-difference threshold (px)
    float moving_ratio_threshold = 0.35f;   // Fraction threshold for suspicious points with motion > patch_px
    int hold_on_frames = 2;                 // Enable the guard after N consecutive trigger frames
    int hold_off_frames = 4;                // Disable the guard after N consecutive non-trigger frames
    int win_size = 21;                      // LK optical-flow window
    int pyr_levels = 3;                     // LK pyramid levels
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
    // When defense is enabled, switch to the ORBSLAM2-style LOST policy, without triggering Atlas multi-map reconstruction.
    bool defense_orbslam2_lost_policy = true;
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
    float loop_cover_reject = 0.05f;      // Reject directly when coverage is below this value
    float loop_cover_no_kl = 0.20f;       // Skip KL and use only safeRatio when coverage is below this value
    float loop_cover_loose = 0.35f;       // Use the loose KL threshold when coverage is below this value
    float loop_kl_loose = 1.0f;           // Loose KL threshold
    float loop_kl_strict = 0.4f;          // Strict KL threshold
    int histogram_bins = 8;
    float histogram_epsilon = 1.0e-4f;
    // Loop-switch integrator: when safeRatio is very high, allow a one-shot fast pass.
    float loop_safe_ratio_boost = 0.9f;    // Use the boost increment when safeRatio >= this value; treat <= tau_safe as off
    float loop_delta_pos_boost = 1.0f;     // Boost increment (recommend >= loop_state_on to implement one-check pass-through)
    float loop_delta_pos = 0.6f;
    float loop_delta_neg = 0.3f;
    float loop_state_on = 0.65f;
    float loop_state_off = 0.3f;
    // Weight scaling coefficients
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
    // Weight-fusion coefficients (normalized and used as exponents to bias a branch)
    float weight_mask = 1.0f;
    float weight_temp = 1.0f;
    // Keyframe safety decision
    bool keyframe_use_safe_ratio = false;
    float keyframe_safe_ratio_min = 0.5f;

    // Loop-match visualization output (independent of use_patch_defense; output is written whenever a path is configured)
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

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_PATCH_DEFENSE_H
