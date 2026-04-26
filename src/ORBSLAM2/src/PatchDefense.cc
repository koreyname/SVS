/**
 * PatchDefense.cc
 * Implements the soft-weighted defense module based on texture contrast and temporal consistency.
 *
 * @author Korey
 * @Date 2025-01-05
 */

#include "PatchDefense.h"
#include "Frame.h"
#include "ORBmatcher.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <sstream>

namespace ORB_SLAM2
{

namespace
{

PatchDefenseConfig gPatchDefenseConfig;
bool gPatchDefenseConfigSet = false;

const float kEps = 1e-6f;

std::vector<std::string> BuildCandidateArgsPaths()
{
    std::vector<std::string> candidates;
    const char* envPath = std::getenv("ORB_SLAM_ARGS");
    if(envPath)
        candidates.emplace_back(envPath);

    std::vector<std::string> relative = {
        "args.yaml",
        "../args.yaml",
        "../../args.yaml",
        "../../../args.yaml",
        "../../../../args.yaml",
        "../../../../../args.yaml"
    };
    candidates.insert(candidates.end(), relative.begin(), relative.end());
    return candidates;
}

template<typename T>
T Clamp(const T &v, const T &minv, const T &maxv)
{
    return std::min(std::max(v, minv), maxv);
}

float ComputeQuantile(std::vector<float> values, float q, float fallback)
{
    if(values.empty())
        return fallback;
    std::sort(values.begin(), values.end());
    q = Clamp(q, 0.f, 1.f);
    const float idx = q * (values.size()-1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    if(lo==hi)
        return values[lo];
    const float ratio = idx - lo;
    return values[lo]*(1.f-ratio) + values[hi]*ratio;
}

} // namespace

bool LoadPatchDefenseConfig(PatchDefenseConfig &config)
{
    PatchDefenseConfig defaults;
    defaults.multi_scale_cells = {4,8,16};
    config = defaults;

    std::vector<std::string> candidates = BuildCandidateArgsPaths();
    cv::FileStorage fs;
    std::string foundPath;
    for(const std::string &candidate : candidates)
    {
        fs.open(candidate, cv::FileStorage::READ);
        if(fs.isOpened())
        {
            foundPath = candidate;
            break;
        }
    }

    if(foundPath.empty())
    {
        std::cerr << "[PatchDefense] args.yaml was not found; using default parameters." << std::endl;
        return false;
    }

    try
    {
        cv::FileNode root = fs.root();
        if(!root["use_patch_defense"].empty())
            config.use_patch_defense = static_cast<int>(root["use_patch_defense"]) != 0;

        cv::FileNode pd = root["patch_defense"];
        if(!pd.empty())
        {
            #define READ_BOOL(name, ref) \
                if(!pd[#name].empty()) (ref) = static_cast<int>(pd[#name]) != 0
            #define READ_INT(name, ref) \
                if(!pd[#name].empty()) (ref) = static_cast<int>(pd[#name])
            #define READ_FLOAT(name, ref) \
                if(!pd[#name].empty()) (ref) = static_cast<float>(pd[#name])

            READ_BOOL(enable_risk_mask, config.enable_risk_mask);
            READ_INT(risk_mask_scope, config.risk_mask_scope);
            READ_BOOL(enable_local_statistics, config.enable_local_statistics);
            READ_BOOL(enable_density_stat, config.enable_density_stat);
            READ_BOOL(enable_descriptor_stat, config.enable_descriptor_stat);
            READ_BOOL(enable_orientation_stat, config.enable_orientation_stat);
            READ_FLOAT(candidate_score_threshold, config.candidate_score_threshold);
            READ_FLOAT(candidate_weight_density, config.candidate_weight_density);
            READ_FLOAT(candidate_weight_descriptor, config.candidate_weight_descriptor);
            READ_FLOAT(candidate_weight_orientation, config.candidate_weight_orientation);
            READ_BOOL(enable_temporal_consistency, config.enable_temporal_consistency);
            READ_INT(temporal_scope, config.temporal_scope);
            READ_BOOL(enable_weighted_frontend, config.enable_weighted_frontend);
            READ_BOOL(enable_weighted_ba, config.enable_weighted_ba);
            READ_BOOL(enable_loop_defense, config.enable_loop_defense);
            READ_BOOL(enable_pyramid_risk, config.enable_pyramid_risk);

            // Motion guard (Tracking-level freeze guard)
            READ_BOOL(enable_motion_guard, config.motion_guard.enable_motion_guard);
            READ_INT(motion_guard_source, config.motion_guard.source);
            READ_INT(motion_guard_mode, config.motion_guard.mode);
            READ_INT(motion_guard_min_keyframes, config.motion_guard.min_keyframes);
            READ_FLOAT(motion_guard_safe_weight_threshold, config.motion_guard.safe_weight_threshold);
            READ_FLOAT(motion_guard_suspect_weight_threshold, config.motion_guard.suspect_weight_threshold);
            READ_INT(motion_guard_min_safe_points, config.motion_guard.min_safe_points);
            READ_INT(motion_guard_min_suspect_points, config.motion_guard.min_suspect_points);
            READ_FLOAT(motion_guard_static_px, config.motion_guard.static_px);
            READ_FLOAT(motion_guard_patch_px, config.motion_guard.patch_px);
            READ_FLOAT(motion_guard_mismatch_px, config.motion_guard.mismatch_px);
            READ_FLOAT(motion_guard_moving_ratio_threshold, config.motion_guard.moving_ratio_threshold);
            READ_INT(motion_guard_hold_on_frames, config.motion_guard.hold_on_frames);
            READ_INT(motion_guard_hold_off_frames, config.motion_guard.hold_off_frames);
            READ_INT(motion_guard_win_size, config.motion_guard.win_size);
            READ_INT(motion_guard_pyr_levels, config.motion_guard.pyr_levels);
            READ_BOOL(log_motion_guard, config.motion_guard.log_motion_guard);

            READ_INT(grid_size, config.grid_size);
            READ_INT(entropy_bins, config.entropy_bins);
            READ_INT(context_radius, config.context_radius);
            READ_FLOAT(lambda_context, config.lambda_context);
            READ_FLOAT(alpha_texture, config.alpha_texture);
            READ_FLOAT(alpha_orientation, config.alpha_orientation);
            READ_INT(orientation_bins, config.orientation_bins);
            READ_FLOAT(orientation_threshold, config.orientation_threshold);
            READ_INT(gaussian_kernel, config.gaussian_kernel);
            READ_FLOAT(gaussian_sigma, config.gaussian_sigma);
            READ_INT(dilation_iterations, config.dilation_iterations);

            READ_FLOAT(density_quantile, config.density_quantile);
            READ_FLOAT(descriptor_quantile, config.descriptor_quantile);
            READ_FLOAT(descriptor_high_quantile, config.descriptor_high_quantile);
            READ_FLOAT(orientation_quantile, config.orientation_quantile);
            READ_INT(desc_min_points, config.desc_min_points);
            READ_INT(orient_min_points, config.orient_min_points);

            READ_FLOAT(ema_alpha_density, config.ema_alpha_density);
            READ_FLOAT(ema_alpha_descriptor, config.ema_alpha_descriptor);
            READ_FLOAT(burst_lambda_density, config.burst_lambda_density);
            READ_FLOAT(burst_lambda_descriptor, config.burst_lambda_descriptor);
            READ_FLOAT(eta_temporal, config.eta_temporal);

            READ_FLOAT(w_min, config.w_min);
            READ_FLOAT(w_floor, config.w_floor);

            READ_FLOAT(safe_weight_threshold, config.safe_weight_threshold);
            READ_FLOAT(tau_safe, config.tau_safe);
            READ_FLOAT(loop_cover_reject, config.loop_cover_reject);
            READ_FLOAT(loop_cover_no_kl, config.loop_cover_no_kl);
            READ_FLOAT(loop_cover_loose, config.loop_cover_loose);
            READ_FLOAT(loop_kl_loose, config.loop_kl_loose);
            READ_FLOAT(loop_kl_strict, config.loop_kl_strict);
            READ_INT(histogram_bins, config.histogram_bins);
            READ_FLOAT(histogram_epsilon, config.histogram_epsilon);
            READ_FLOAT(loop_safe_ratio_boost, config.loop_safe_ratio_boost);
            READ_FLOAT(loop_delta_pos_boost, config.loop_delta_pos_boost);
            READ_FLOAT(loop_delta_pos, config.loop_delta_pos);
            READ_FLOAT(loop_delta_neg, config.loop_delta_neg);
            READ_FLOAT(loop_state_on, config.loop_state_on);
            READ_FLOAT(loop_state_off, config.loop_state_off);

            READ_FLOAT(k_mask_weight, config.k_mask_weight);
            READ_FLOAT(k_temp_weight, config.k_temp_weight);
            READ_FLOAT(k_risk_mask, config.k_risk_mask);

            READ_BOOL(logging, config.logging);
            READ_BOOL(log_risk_mask, config.log_risk_mask);
            READ_BOOL(log_density, config.log_density);
            READ_BOOL(log_descriptor, config.log_descriptor);
            READ_BOOL(log_orientation, config.log_orientation);
            READ_BOOL(log_candidate_gate, config.log_candidate_gate);
            READ_BOOL(log_temporal, config.log_temporal);
            READ_BOOL(log_loop_defense, config.log_loop_defense);

            READ_FLOAT(weight_mask, config.weight_mask);
            READ_FLOAT(weight_temp, config.weight_temp);

            READ_BOOL(keyframe_use_safe_ratio, config.keyframe_use_safe_ratio);
            READ_FLOAT(keyframe_safe_ratio_min, config.keyframe_safe_ratio_min);

            if(!pd["loop_match_output_dir"].empty())
                config.loop_match_output_dir = std::string(pd["loop_match_output_dir"]);

            cv::FileNode scalesNode = pd["multi_scale_cells"];
            if(!scalesNode.empty() && scalesNode.isSeq())
            {
                config.multi_scale_cells.clear();
                for(auto it = scalesNode.begin(); it != scalesNode.end(); ++it)
                    config.multi_scale_cells.push_back(static_cast<int>(*it));
            }
            #undef READ_BOOL
            #undef READ_INT
            #undef READ_FLOAT
        }
    }
    catch(const cv::Exception &ex)
    {
        std::cerr << "[PatchDefense] Failed to parse args.yaml: " << ex.what() << std::endl;
        return false;
    }

    if(config.multi_scale_cells.empty())
        config.multi_scale_cells = {4,8,16};

    return true;
}

void SetPatchDefenseConfig(const PatchDefenseConfig &config)
{
    gPatchDefenseConfig = config;
    gPatchDefenseConfigSet = true;
}

const PatchDefenseConfig& GetPatchDefenseConfig()
{
    return gPatchDefenseConfig;
}

PatchDefense::PatchDefense() = default;

void PatchDefense::SetConfig(const PatchDefenseConfig &config)
{
    mConfig = config;
    mbEnabled = mConfig.use_patch_defense;
    mbConfigured = false;
}

void PatchDefense::Configure(const PatchDefenseConfig &config, const cv::Size &imageSize, int nLevels, const std::vector<float> &scaleFactors)
{
    mConfig = config;
    if(mConfig.multi_scale_cells.empty())
        mConfig.multi_scale_cells = {4,8,16};
    mBaseSize = imageSize;
    mnLevels = nLevels;
    mScaleFactors = scaleFactors;
    mInvScaleFactors.resize(mScaleFactors.size());
    for(size_t i=0; i<mScaleFactors.size(); ++i)
        mInvScaleFactors[i] = 1.0f / mScaleFactors[i];

    mRiskPyramid.resize(mnLevels);
    mTemporalStates.clear();
    mTemporalStates.resize(mnLevels);
    for(int level=0; level<mnLevels; ++level)
    {
        mTemporalStates[level].resize(mConfig.multi_scale_cells.size());
        for(size_t si=0; si<mConfig.multi_scale_cells.size(); ++si)
        {
            const int cells = mConfig.multi_scale_cells[si] * mConfig.multi_scale_cells[si];
            mTemporalStates[level][si].assign(cells, GridTemporalState());
        }
    }

    mGridCols = std::max(1, static_cast<int>(std::ceil(static_cast<float>(mBaseSize.width)/mConfig.grid_size)));
    mGridRows = std::max(1, static_cast<int>(std::ceil(static_cast<float>(mBaseSize.height)/mConfig.grid_size)));

    mbConfigured = true;
    mbEnabled = mConfig.use_patch_defense;
}

void PatchDefense::ProcessFrame(Frame &frame, const cv::Mat &gray)
{
    if(!mbEnabled)
    {
        mLastMeanRisk = 0.f;
        mLastSafeRatio = 1.f;
        return;
    }

    if(!mbConfigured)
    {
        Configure(mConfig, gray.size(), frame.mnScaleLevels, frame.mvScaleFactors);
    }

    if(frame.N<=0)
    {
        mLastMeanRisk = 0.f;
        mLastSafeRatio = 1.f;
        return;
    }

    const bool needLocalStats = mConfig.enable_local_statistics || mConfig.enable_temporal_consistency;
    if(needLocalStats)
        ComputeLocalStatistics(frame);
    else
    {
        mDensityRisks.assign(frame.N, 0.f);
        mDescriptorRisks.assign(frame.N, 0.f);
        mOrientationRisks.assign(frame.N, 0.f);
        mTemporalRaw.assign(frame.N, 0.f);
    }

    std::vector<uint8_t> candidates(frame.N, 0);
    bool hasCandidate = false;
    int candidateCount = 0;
    float candScoreMin = 1.f;
    float candScoreMax = 0.f;
    double candScoreSum = 0.0;
    const bool candidateGateEnabled = mConfig.enable_local_statistics &&
        (mConfig.enable_density_stat || mConfig.enable_descriptor_stat || mConfig.enable_orientation_stat);
    if(candidateGateEnabled)
    {
        const float thr = Clamp(mConfig.candidate_score_threshold, 0.f, 1.f);
        const float wD = std::max(0.f, mConfig.candidate_weight_density);
        const float wS = std::max(0.f, mConfig.candidate_weight_descriptor);
        const float wO = std::max(0.f, mConfig.candidate_weight_orientation);
        for(int i=0; i<frame.N; ++i)
        {
            float sumW = 0.f;
            float sumScore = 0.f;
            if(mConfig.enable_local_statistics && mConfig.enable_density_stat && i < static_cast<int>(mDensityRisks.size()))
            {
                sumW += wD;
                sumScore += wD * Clamp(mDensityRisks[i], 0.f, 1.f);
            }
            if(mConfig.enable_local_statistics && mConfig.enable_descriptor_stat && i < static_cast<int>(mDescriptorRisks.size()))
            {
                sumW += wS;
                sumScore += wS * Clamp(mDescriptorRisks[i], 0.f, 1.f);
            }
            if(mConfig.enable_local_statistics && mConfig.enable_orientation_stat && i < static_cast<int>(mOrientationRisks.size()))
            {
                sumW += wO;
                sumScore += wO * Clamp(mOrientationRisks[i], 0.f, 1.f);
            }

            const float score = (sumW > 0.f) ? (sumScore / sumW) : 0.f;
            const bool isCandidate = score >= thr;
            candidates[i] = isCandidate ? 1 : 0;
            hasCandidate = hasCandidate || isCandidate;
            if(mConfig.log_candidate_gate || mConfig.logging)
            {
                candScoreMin = std::min(candScoreMin, score);
                candScoreMax = std::max(candScoreMax, score);
                candScoreSum += static_cast<double>(score);
            }
            if(isCandidate)
                candidateCount++;
        }
    }

    const bool shouldRunRiskMask = mConfig.enable_risk_mask && (!candidateGateEnabled || hasCandidate);
    if(shouldRunRiskMask)
    {
        const bool useLocalRoi = (mConfig.risk_mask_scope == 1) && candidateGateEnabled && hasCandidate;
        if(useLocalRoi)
            ComputeRiskMaps(gray, &candidates, &frame);
        else
            ComputeRiskMaps(gray, nullptr, nullptr);
        AssignRiskToKeypoints(frame, candidateGateEnabled ? &candidates : nullptr);
    }
    else
    {
        frame.mvPatchRisk.assign(frame.N, 0.f);
        mLastMeanRisk = 0.f;
        mLastRiskMask.release();
        for(int level=0; level<mnLevels; ++level)
            mRiskPyramid[level].release();
    }

    if(mConfig.enable_temporal_consistency && needLocalStats)
    {
        ComputeTemporalWeights(frame);
        if(mConfig.temporal_scope == 1 && hasCandidate)
        {
            for(int i=0; i<frame.N && i < static_cast<int>(mTemporalWeights.size()); ++i)
            {
                if(!candidates[i])
                    mTemporalWeights[i] = 1.f;
            }
        }
        else if(mConfig.temporal_scope == 1 && !hasCandidate)
        {
            for(size_t i=0; i<mTemporalWeights.size(); ++i)
                mTemporalWeights[i] = 1.f;
        }
    }
    else
        mTemporalWeights.assign(frame.N, 1.f);

    CombineWeights(frame);

    const bool logAny = mConfig.logging || mConfig.log_risk_mask || mConfig.log_density ||
                        mConfig.log_descriptor || mConfig.log_orientation || mConfig.log_candidate_gate ||
                        mConfig.log_temporal;
    if(logAny)
    {
        std::cout << "[PatchDefense] Frame " << frame.mnId
                  << " safeRatio=" << mLastSafeRatio
                  << " (" << mLastSafeCount << "/" << frame.N << ")";
        if(mConfig.log_candidate_gate || mConfig.logging)
        {
            if(candidateGateEnabled)
            {
                const double mean = candScoreSum / std::max(1, frame.N);
                std::cout << " cand=" << candidateCount << "/" << frame.N
                          << " candThr=" << Clamp(mConfig.candidate_score_threshold, 0.f, 1.f)
                          << " score[min/mean/max]=" << candScoreMin
                          << "/" << mean
                          << "/" << candScoreMax;
            }
            else
            {
                std::cout << " cand=0/" << frame.N << " gate=off";
            }
        }
        if(mConfig.log_risk_mask || mConfig.logging)
            std::cout << " meanRisk=" << mLastMeanRisk;
        if(mConfig.log_density && mConfig.enable_density_stat && mConfig.enable_local_statistics)
        {
            std::cout << " density[min/mean/max]="
                      << mLastDensityMin << "/"
                      << mLastDensityMean << "/"
                      << mLastDensityMax
                      << " highCount=" << mLastDensityHighCount
                      << "/" << mLastPointCount;
            for(const auto &line : mLastDensityLogs)
                std::cout << " " << line;
        }
        if(mConfig.log_descriptor && mConfig.enable_descriptor_stat && mConfig.enable_local_statistics)
        {
            std::cout << " descVar[min/mean/max]="
                      << mLastDescriptorMin << "/"
                      << mLastDescriptorMean << "/"
                      << mLastDescriptorMax
                      << " highCount=" << mLastDescriptorHighCount
                      << "/" << mLastPointCount;
            for(const auto &line : mLastDescriptorLogs)
                std::cout << " " << line;
        }
        if(mConfig.log_orientation && mConfig.enable_orientation_stat && mConfig.enable_local_statistics)
        {
            std::cout << " orient[min/mean/max]="
                      << mLastOrientationMin << "/"
                      << mLastOrientationMean << "/"
                      << mLastOrientationMax
                      << " highCount=" << mLastOrientationHighCount
                      << "/" << mLastPointCount;
            for(const auto &line : mLastOrientationLogs)
                std::cout << " " << line;
        }
        if(mConfig.log_temporal && mConfig.enable_temporal_consistency)
        {
            double rawSum = 0.0;
            double wSum = 0.0;
            float rawMax = 0.f;
            float wMin = 1.f;
            int wLowCount = 0;
            const float wLowThr = 0.5f;
            for(int i=0; i<frame.N; ++i)
            {
                const float raw = (i < static_cast<int>(mTemporalRaw.size())) ? mTemporalRaw[i] : 0.f;
                const float w = (i < static_cast<int>(mTemporalWeights.size())) ? mTemporalWeights[i] : 1.f;
                rawSum += raw;
                wSum += w;
                rawMax = std::max(rawMax, raw);
                wMin = std::min(wMin, w);
                if(w < wLowThr)
                    wLowCount++;
            }
            const float rawMean = static_cast<float>(rawSum / std::max(1, frame.N));
            const float wMean = static_cast<float>(wSum / std::max(1, frame.N));
            std::cout << " temp[eta=" << mConfig.eta_temporal
                      << " rawMean/max=" << rawMean << "/" << rawMax
                      << " wMin/mean=" << wMin << "/" << wMean
                      << " w<" << wLowThr << "=" << wLowCount << "/" << frame.N << "]";
        }
        std::cout << std::endl;
    }
}

namespace
{

inline bool AnyCandidate(const std::vector<uint8_t> &candidates)
{
    for(uint8_t v : candidates)
        if(v)
            return true;
    return false;
}

inline void ExpandCellMask(std::vector<uint8_t> &mask, int cols, int rows, int radius)
{
    if(radius <= 0)
        return;
    std::vector<uint8_t> copy = mask;
    for(int y=0; y<rows; ++y)
    {
        for(int x=0; x<cols; ++x)
        {
            if(!copy[y*cols + x])
                continue;
            for(int dy=-radius; dy<=radius; ++dy)
            {
                for(int dx=-radius; dx<=radius; ++dx)
                {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if(nx<0 || ny<0 || nx>=cols || ny>=rows)
                        continue;
                    mask[ny*cols + nx] = 1;
                }
            }
        }
    }
}

} // namespace

void PatchDefense::ComputeRiskMaps(const cv::Mat &gray, const std::vector<uint8_t> *candidates, const Frame *frame)
{
    cv::Mat gradX, gradY;
    cv::Sobel(gray, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(gray, gradY, CV_32F, 0, 1, 3);
    cv::Mat magnitude, angle;
    cv::cartToPolar(gradX, gradY, magnitude, angle, false);
    angle -= CV_PI;

    std::vector<float> entropyGrid(mGridCols * mGridRows, 0.f);
    std::vector<float> orientScore(mGridCols * mGridRows, 0.f);

    std::vector<uint8_t> seedCells;
    std::vector<uint8_t> fillCells;
    std::vector<uint8_t> computeCells;
    cv::Mat roiMask;
    const bool useLocalRoi = (mConfig.risk_mask_scope == 1) && candidates && frame && AnyCandidate(*candidates);
    if(useLocalRoi)
    {
        seedCells.assign(mGridCols*mGridRows, 0);
        for(int i=0; i<frame->N; ++i)
        {
            if(i >= static_cast<int>(candidates->size()) || !(*candidates)[i])
                continue;
            const cv::KeyPoint &kp = frame->mvKeysUn[i];
            const int gx = Clamp(static_cast<int>(kp.pt.x / std::max(1, mConfig.grid_size)), 0, mGridCols-1);
            const int gy = Clamp(static_cast<int>(kp.pt.y / std::max(1, mConfig.grid_size)), 0, mGridRows-1);
            seedCells[gy*mGridCols + gx] = 1;
        }

        fillCells = seedCells;
        ExpandCellMask(fillCells, mGridCols, mGridRows, std::max(0, mConfig.context_radius));
        computeCells = fillCells;
        ExpandCellMask(computeCells, mGridCols, mGridRows, std::max(0, mConfig.context_radius));

        roiMask = cv::Mat::zeros(gray.size(), CV_8U);
        for(int gy=0; gy<mGridRows; ++gy)
        {
            int y0 = gy*mConfig.grid_size;
            int y1 = std::min(gray.rows, y0 + mConfig.grid_size);
            for(int gx=0; gx<mGridCols; ++gx)
            {
                const size_t idx = gy*mGridCols + gx;
                if(!fillCells[idx])
                    continue;
                int x0 = gx*mConfig.grid_size;
                int x1 = std::min(gray.cols, x0 + mConfig.grid_size);
                cv::Rect roi(x0, y0, x1-x0, y1-y0);
                roiMask(roi).setTo(255);
            }
        }
    }

    const int bins = std::max(1, mConfig.entropy_bins);
    const int orientBins = std::max(1, mConfig.orientation_bins);
    const int binWidth = std::max(1, 256/bins);

    for(int gy=0; gy<mGridRows; ++gy)
    {
        int y0 = gy*mConfig.grid_size;
        int y1 = std::min(gray.rows, y0 + mConfig.grid_size);
        for(int gx=0; gx<mGridCols; ++gx)
        {
            const size_t idx = gy*mGridCols + gx;
            if(useLocalRoi && !computeCells[idx])
                continue;
            int x0 = gx*mConfig.grid_size;
            int x1 = std::min(gray.cols, x0 + mConfig.grid_size);
            std::vector<int> hist(bins,0);
            std::vector<float> orientHist(orientBins,0.f);
            float orientTotal = 0.f;
            for(int y=y0; y<y1; ++y)
            {
                const uchar* row = gray.ptr<uchar>(y);
                const float* magRow = magnitude.ptr<float>(y);
                const float* angRow = angle.ptr<float>(y);
                for(int x=x0; x<x1; ++x)
                {
                    const int bin = Clamp(static_cast<int>(row[x]/binWidth),0,bins-1);
                    hist[bin]++;
                    const float mag = magRow[x];
                    const float theta = angRow[x];
                    float normalized = (theta + CV_PI) / (2.0f * CV_PI);
                    normalized = Clamp(normalized, 0.f, 0.999999f);
                    int obin = Clamp(static_cast<int>(normalized * orientBins), 0, orientBins-1);
                    orientHist[obin] += mag + kEps;
                    orientTotal += mag + kEps;
                }
            }
            float entropy = 0.f;
            const float total = std::max(1, (x1-x0)*(y1-y0));
            for(int count : hist)
            {
                if(count<=0) continue;
                const float p = count/total;
                entropy -= p * std::log(std::max(p, kEps));
            }
            float maxProb = 1.f/static_cast<float>(orientBins);
            if(orientTotal>0)
            {
                for(float v : orientHist)
                    maxProb = std::max(maxProb, v/orientTotal);
            }
            const float C = orientBins * maxProb;
            entropyGrid[idx] = entropy;
            orientScore[idx] = std::max(C - mConfig.orientation_threshold, 0.f);
        }
    }

    std::vector<float> contextEntropy(entropyGrid.size(),0.f);
    for(int gy=0; gy<mGridRows; ++gy)
    {
        for(int gx=0; gx<mGridCols; ++gx)
        {
            const size_t idx = gy*mGridCols + gx;
            if(useLocalRoi && !fillCells[idx])
            {
                contextEntropy[idx] = entropyGrid[idx];
                continue;
            }
            float sum = 0.f;
            int count = 0;
            for(int dy=-mConfig.context_radius; dy<=mConfig.context_radius; ++dy)
            {
                for(int dx=-mConfig.context_radius; dx<=mConfig.context_radius; ++dx)
                {
                    if(dx==0 && dy==0) continue;
                    int ny = gy + dy;
                    int nx = gx + dx;
                    if(nx<0 || ny<0 || nx>=mGridCols || ny>=mGridRows)
                        continue;
                    sum += entropyGrid[ny*mGridCols + nx];
                    ++count;
                }
            }
            contextEntropy[idx] = (count>0) ? sum / count : entropyGrid[idx];
        }
    }

    cv::Mat riskBase(gray.size(), CV_32F);
    for(int gy=0; gy<mGridRows; ++gy)
    {
        int y0 = gy*mConfig.grid_size;
        int y1 = std::min(gray.rows, y0 + mConfig.grid_size);
        for(int gx=0; gx<mGridCols; ++gx)
        {
            int x0 = gx*mConfig.grid_size;
            int x1 = std::min(gray.cols, x0 + mConfig.grid_size);
            const size_t idx = gy*mGridCols + gx;
            if(useLocalRoi && !fillCells[idx])
            {
                cv::Rect roi(x0, y0, x1-x0, y1-y0);
                riskBase(roi).setTo(0.f);
                continue;
            }
            const float S_anom = std::max(entropyGrid[idx] - mConfig.lambda_context * contextEntropy[idx], 0.f);
            const float S_ori = orientScore[idx];
            float risk = 1.f - std::exp(-mConfig.alpha_texture * S_anom - mConfig.alpha_orientation * S_ori);
            cv::Rect roi(x0, y0, x1-x0, y1-y0);
            riskBase(roi).setTo(risk);
        }
    }

    if(mConfig.gaussian_kernel > 1)
    {
        const int ksize = (mConfig.gaussian_kernel % 2 == 0) ? mConfig.gaussian_kernel+1 : mConfig.gaussian_kernel;
        cv::GaussianBlur(riskBase, riskBase, cv::Size(ksize, ksize), mConfig.gaussian_sigma);
    }
    if(mConfig.dilation_iterations > 0)
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        cv::dilate(riskBase, riskBase, kernel, cv::Point(-1,-1), mConfig.dilation_iterations);
        cv::threshold(riskBase, riskBase, 1.f, 1.f, cv::THRESH_TRUNC);
    }

    if(useLocalRoi && !roiMask.empty())
    {
        cv::Mat inv;
        cv::bitwise_not(roiMask, inv);
        riskBase.setTo(0.f, inv);
    }

    riskBase.copyTo(mLastRiskMask);

    for(int level=0; level<mnLevels; ++level)
    {
        if(mConfig.enable_pyramid_risk)
        {
            const float scale = mScaleFactors[level];
            const int width = std::max(1, static_cast<int>(std::round(mBaseSize.width / scale)));
            const int height = std::max(1, static_cast<int>(std::round(mBaseSize.height / scale)));
            cv::resize(riskBase, mRiskPyramid[level], cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
        }
        else
        {
            riskBase.copyTo(mRiskPyramid[level]);
        }
    }
}

void PatchDefense::AssignRiskToKeypoints(Frame &frame, const std::vector<uint8_t> *candidates)
{
    float riskSum = 0.f;
    frame.mvPatchRisk.assign(frame.N, 0.f);
    for(int i=0; i<frame.N; ++i)
    {
        if(candidates && i < static_cast<int>(candidates->size()) && !(*candidates)[i])
        {
            frame.mvPatchRisk[i] = 0.f;
            continue;
        }
        const int octave = Clamp(frame.mvKeysUn[i].octave, 0, mnLevels-1);
        float risk = SampleRiskAt(frame, i, octave);
        frame.mvPatchRisk[i] = Clamp(risk, 0.f, 1.f);
        riskSum += frame.mvPatchRisk[i];
    }
    mLastMeanRisk = riskSum / std::max(1, frame.N);
}

float PatchDefense::SampleRiskAt(const Frame &frame, size_t idx, int octave) const
{
    if(octave<0 || octave>=mnLevels)
        return 0.f;
    const int sampleLevel = mConfig.enable_pyramid_risk ? octave : 0;
    const cv::Mat &risk = mRiskPyramid[sampleLevel];
    if(risk.empty())
        return 0.f;
    const cv::KeyPoint &kp = frame.mvKeysUn[idx];
    float invScale = mConfig.enable_pyramid_risk ? mInvScaleFactors[octave] : 1.f;
    int x = static_cast<int>(std::round(kp.pt.x * invScale));
    int y = static_cast<int>(std::round(kp.pt.y * invScale));
    x = Clamp(x, 0, risk.cols-1);
    y = Clamp(y, 0, risk.rows-1);
    return risk.at<float>(y,x);
}

void PatchDefense::ComputeLocalStatistics(Frame &frame)
{
    mDensityRisks.assign(frame.N, 0.f);
    mDescriptorRisks.assign(frame.N, 0.f);
    mOrientationRisks.assign(frame.N, 0.f);
    mTemporalRaw.assign(frame.N, 0.f);
    std::vector<float> densityAggregate;
    std::vector<float> descriptorAggregate;
    std::vector<float> orientationAggregate;
    mLastDensityHighCount = 0;
    mLastDescriptorHighCount = 0;
    mLastOrientationHighCount = 0;
    mLastPointCount = frame.N;
    mLastDensityLogs.clear();
    mLastDescriptorLogs.clear();
    mLastOrientationLogs.clear();

    std::vector<std::vector<int>> levelPoints(mnLevels);
    for(int i=0; i<frame.N; ++i)
    {
        int level = Clamp(frame.mvKeysUn[i].octave, 0, mnLevels-1);
        levelPoints[level].push_back(i);
    }

    for(int level=0; level<mnLevels; ++level)
    {
        const auto &indices = levelPoints[level];
        if(indices.empty())
            continue;
        const cv::Mat &riskLevel = mRiskPyramid[level];
        // Risk maps may be empty when the risk mask is disabled; in that case, fall back to the image-pyramid size.
        const float scale = mScaleFactors[level];
        const int fallbackWidth = std::max(1, static_cast<int>(std::round(mBaseSize.width / scale)));
        const int fallbackHeight = std::max(1, static_cast<int>(std::round(mBaseSize.height / scale)));
        const int levelWidth = riskLevel.empty() ? fallbackWidth : riskLevel.cols;
        const int levelHeight = riskLevel.empty() ? fallbackHeight : riskLevel.rows;

        for(size_t sIdx=0; sIdx<mConfig.multi_scale_cells.size(); ++sIdx)
        {
            const int s = mConfig.multi_scale_cells[sIdx];
            const int cellCount = s*s;
            std::vector<std::vector<int>> cellPoints(cellCount);

            for(int idx : indices)
            {
                const cv::KeyPoint &kp = frame.mvKeysUn[idx];
                const float invScale = mInvScaleFactors[level];
                float xLevel = kp.pt.x * invScale;
                float yLevel = kp.pt.y * invScale;
                float nx = Clamp(xLevel / std::max(1.f, static_cast<float>(levelWidth)), 0.f, 0.999999f);
                float ny = Clamp(yLevel / std::max(1.f, static_cast<float>(levelHeight)), 0.f, 0.999999f);
                int cx = Clamp(static_cast<int>(nx * s), 0, s-1);
                int cy = Clamp(static_cast<int>(ny * s), 0, s-1);
                const int cellIdx = cy*s + cx;
                cellPoints[cellIdx].push_back(idx);
            }

            std::vector<float> densityRatios(cellCount, 0.f);
            std::vector<float> descVars(cellCount, 0.f);
            std::vector<float> orientConfs(cellCount, 0.f);
            std::vector<float> temporalRisks(cellCount, 0.f);

            const float lambda = std::max(1e-3f, static_cast<float>(indices.size()) / cellCount);
            const int descriptorCols = frame.mDescriptors.cols;

            std::vector<float> densityValues;
            std::vector<float> descriptorValues;
            std::vector<float> orientValues;
            densityValues.reserve(cellCount);
            descriptorValues.reserve(cellCount);
            orientValues.reserve(cellCount);
            std::vector<float> descriptorValuesFiltered;
            std::vector<float> orientationValuesFiltered;

            for(int c=0; c<cellCount; ++c)
            {
                const auto &pts = cellPoints[c];
                const float d = pts.size() / lambda;
                densityRatios[c] = d;
                densityValues.push_back(d);

                if(static_cast<int>(pts.size()) < mConfig.desc_min_points)
                {
                    descVars[c] = 0.f;
                    descriptorValues.push_back(0.f);
                    // orientation will be handled below
                }
                else
                {
                    // descriptor variance
                    int medoid = pts[0];
                    int bestSum = std::numeric_limits<int>::max();
                    for(int idx : pts)
                    {
                        int sumDist = 0;
                        for(int other : pts)
                            sumDist += ORBmatcher::DescriptorDistance(frame.mDescriptors.row(idx), frame.mDescriptors.row(other));
                        if(sumDist < bestSum)
                        {
                            bestSum = sumDist;
                            medoid = idx;
                        }
                    }

                    float var = 0.f;
                    for(int idx : pts)
                    {
                        const int dist = ORBmatcher::DescriptorDistance(frame.mDescriptors.row(idx), frame.mDescriptors.row(medoid));
                        var += static_cast<float>(dist*dist);
                    }
                    var /= pts.size();
                    descVars[c] = var;
                    descriptorValues.push_back(var);
                    if(var > 1e-3f)
                        descriptorValuesFiltered.push_back(var);
                }

                // orientation concentration
                float sumCos = 0.f;
                float sumSin = 0.f;
                if(static_cast<int>(pts.size()) >= mConfig.orient_min_points)
                {
                    for(int idx : pts)
                    {
                        const float theta = frame.mvKeysUn[idx].angle * static_cast<float>(CV_PI/180.0);
                        sumCos += std::cos(theta);
                        sumSin += std::sin(theta);
                    }
                    const float R = std::sqrt(sumCos*sumCos + sumSin*sumSin) / pts.size();
                    orientConfs[c] = R;
                    orientValues.push_back(R);
                    if(R > 1e-3f)
                        orientationValuesFiltered.push_back(R);
                }
            }

            std::vector<float> densityValuesFiltered;
            densityValuesFiltered.reserve(densityValues.size());
            for(float v : densityValues)
            {
                if(v > 1e-3f)
                    densityValuesFiltered.push_back(v);
            }
            const std::vector<float> &densityPool = densityValuesFiltered.empty() ? densityValues : densityValuesFiltered;
            const std::vector<float> &descriptorPool = descriptorValuesFiltered.empty() ? descriptorValues : descriptorValuesFiltered;
            const float Td = ComputeQuantile(densityPool, mConfig.density_quantile, 1.f);
            const float TdescLow = ComputeQuantile(descriptorPool, mConfig.descriptor_quantile, 0.f);
            const float TdescHigh = ComputeQuantile(descriptorPool, mConfig.descriptor_high_quantile, 0.f);
            const std::vector<float> &orientationPool = orientationValuesFiltered.empty() ? orientValues : orientationValuesFiltered;
            const float Ttheta = ComputeQuantile(orientationPool, mConfig.orientation_quantile, 0.f);
            const float dmax = *std::max_element(densityRatios.begin(), densityRatios.end());
            const bool validDensity = mConfig.enable_density_stat && Td > 1e-3f && dmax > Td;
            const float descMax = descriptorValues.empty() ? 0.f : *std::max_element(descriptorValues.begin(), descriptorValues.end());
            const bool validDescLow = mConfig.enable_descriptor_stat && TdescLow > kEps;
            const bool validDescHigh = mConfig.enable_descriptor_stat && TdescHigh > kEps && descMax > TdescHigh;
            densityAggregate.insert(densityAggregate.end(), densityPool.begin(), densityPool.end());
            descriptorAggregate.insert(descriptorAggregate.end(), descriptorValues.begin(), descriptorValues.end());
            orientationAggregate.insert(orientationAggregate.end(), orientValues.begin(), orientValues.end());
            int cellsHigh = 0;
            double cellsSum = 0.0;
            float cellsMin = std::numeric_limits<float>::infinity();
            float cellsMax = 0.f;

            for(int c=0; c<cellCount; ++c)
            {
                float rD = 0.f;
                if(validDensity && densityRatios[c] > Td)
                {
                    const float denom = dmax - Td;
                    if(denom > 1e-3f)
                        rD = (densityRatios[c] - Td) / denom;
                    else
                        rD = 0.f; // When the threshold is close to the maximum value, do not saturate the risk.
                }

                float rSigma = 0.f;
                if(validDescLow && descVars[c] < TdescLow)
                    rSigma = std::max(rSigma, (TdescLow - descVars[c]) / (TdescLow + kEps));
                if(validDescHigh && descVars[c] > TdescHigh)
                {
                    const float denom = descMax - TdescHigh;
                    if(denom > kEps)
                        rSigma = std::max(rSigma, (descVars[c] - TdescHigh) / denom);
                }

                float rTheta = 0.f;
                if(mConfig.enable_orientation_stat && orientConfs[c] > Ttheta && Ttheta < 1.f - 1e-3f)
                    rTheta = (orientConfs[c] - Ttheta) / (1.f - Ttheta);

                // temporal risk
                GridTemporalState &state = mTemporalStates[level][sIdx][c];
                if(!state.initialized)
                {
                    state.mu_density = densityRatios[c];
                    state.var_density = 1e-3f;
                    state.mu_descriptor = descVars[c];
                    state.var_descriptor = 1e-3f;
                    state.initialized = true;
                }
                else
                {
                    state.mu_density = mConfig.ema_alpha_density * state.mu_density +
                        (1.f - mConfig.ema_alpha_density) * densityRatios[c];
                    const float diffD = densityRatios[c] - state.mu_density;
                    state.var_density = mConfig.ema_alpha_density * state.var_density +
                        (1.f - mConfig.ema_alpha_density) * diffD * diffD;

                    state.mu_descriptor = mConfig.ema_alpha_descriptor * state.mu_descriptor +
                        (1.f - mConfig.ema_alpha_descriptor) * descVars[c];
                    const float diffSigma = descVars[c] - state.mu_descriptor;
                    state.var_descriptor = mConfig.ema_alpha_descriptor * state.var_descriptor +
                        (1.f - mConfig.ema_alpha_descriptor) * diffSigma * diffSigma;
                }

                const float relMean = std::max((densityRatios[c] - state.mu_density) / (std::fabs(state.mu_density) + 1e-3f), 0.f);
                const float relStd = std::max((densityRatios[c] - state.mu_density) /
                                              std::sqrt(state.var_density + 1e-3f), 0.f);
                const float relSigmaMean = std::max((state.mu_descriptor - descVars[c]) / (std::fabs(state.mu_descriptor) + 1e-3f), 0.f);
                const float relSigmaStd = std::max((state.mu_descriptor - descVars[c]) /
                                                   std::sqrt(state.var_descriptor + 1e-3f), 0.f);

                temporalRisks[c] = relMean + mConfig.burst_lambda_density * relStd +
                                   relSigmaMean + mConfig.burst_lambda_descriptor * relSigmaStd;

                for(int idx : cellPoints[c])
                {
                    mDensityRisks[idx] = std::max(mDensityRisks[idx], rD);
                    mDescriptorRisks[idx] = std::max(mDescriptorRisks[idx], rSigma);
                    mOrientationRisks[idx] = std::max(mOrientationRisks[idx], rTheta);
                    mTemporalRaw[idx] = std::max(mTemporalRaw[idx], temporalRisks[c]);
                }
            }

            if(validDensity && !densityRatios.empty())
            {
                for(float d : densityRatios)
                {
                    cellsSum += d;
                    cellsMin = std::min(cellsMin, d);
                    cellsMax = std::max(cellsMax, d);
                    if(d > Td)
                        cellsHigh++;
                }
                const float cellsMean = static_cast<float>(cellsSum / std::max(1, static_cast<int>(densityRatios.size())));
                std::ostringstream oss;
                oss << "[density] s=" << s
                    << " cellsHigh=" << cellsHigh << "/" << cellCount
                    << " dmin/mean/max=" << cellsMin << "/" << cellsMean << "/" << cellsMax
                    << " Td=" << Td << " dmax=" << dmax;
                mLastDensityLogs.push_back(oss.str());
            }

            if(mConfig.enable_orientation_stat && !orientValues.empty())
            {
                int cellsHighOri = 0;
                double oriSum = 0.0;
                float oriMin = std::numeric_limits<float>::infinity();
                float oriMax = 0.f;
                for(float v : orientConfs)
                {
                    oriSum += v;
                    oriMin = std::min(oriMin, v);
                    oriMax = std::max(oriMax, v);
                    if(v > Ttheta)
                        cellsHighOri++;
                }
                const float oriMean = static_cast<float>(oriSum / std::max(1, cellCount));
                std::ostringstream ossOri;
                ossOri << "[orient] s=" << s
                       << " cellsHigh=" << cellsHighOri << "/" << cellCount
                       << " cmin/mean/max=" << oriMin << "/" << oriMean << "/" << oriMax
                       << " Tori=" << Ttheta;
                mLastOrientationLogs.push_back(ossOri.str());
            }

            if(mConfig.enable_descriptor_stat && !descriptorValues.empty())
            {
                int cellsLow = 0;
                int cellsHigh = 0;
                double descSum = 0.0;
                float descMin = std::numeric_limits<float>::infinity();
                float descMaxLocal = 0.f;
                for(float v : descVars)
                {
                    descSum += v;
                    descMin = std::min(descMin, v);
                    descMaxLocal = std::max(descMaxLocal, v);
                    if(validDescLow && v < TdescLow)
                        cellsLow++;
                    if(validDescHigh && v > TdescHigh)
                        cellsHigh++;
                }
                const float descMean = static_cast<float>(descSum / std::max(1, cellCount));
                std::ostringstream ossDesc;
                ossDesc << "[desc] s=" << s
                        << " cellsLow=" << cellsLow << "/" << cellCount
                        << " cellsHigh=" << cellsHigh << "/" << cellCount
                        << " varmin/mean/max=" << descMin << "/" << descMean << "/" << descMaxLocal
                        << " Tlow=" << TdescLow << " Thigh=" << TdescHigh;
                mLastDescriptorLogs.push_back(ossDesc.str());
            }
        }
    }

    if(!densityAggregate.empty())
    {
        mLastDensityMin = *std::min_element(densityAggregate.begin(), densityAggregate.end());
        mLastDensityMax = *std::max_element(densityAggregate.begin(), densityAggregate.end());
        const double sum = std::accumulate(densityAggregate.begin(), densityAggregate.end(), 0.0);
        mLastDensityMean = static_cast<float>(sum / std::max<size_t>(1, densityAggregate.size()));
    }
    else
    {
        mLastDensityMin = mLastDensityMax = mLastDensityMean = 0.f;
    }
    if(!descriptorAggregate.empty())
    {
        mLastDescriptorMin = *std::min_element(descriptorAggregate.begin(), descriptorAggregate.end());
        mLastDescriptorMax = *std::max_element(descriptorAggregate.begin(), descriptorAggregate.end());
        const double sumDesc = std::accumulate(descriptorAggregate.begin(), descriptorAggregate.end(), 0.0);
        mLastDescriptorMean = static_cast<float>(sumDesc / std::max<size_t>(1, descriptorAggregate.size()));
    }
    else
    {
        mLastDescriptorMin = mLastDescriptorMax = mLastDescriptorMean = 0.f;
    }
    if(!orientationAggregate.empty())
    {
        mLastOrientationMin = *std::min_element(orientationAggregate.begin(), orientationAggregate.end());
        mLastOrientationMax = *std::max_element(orientationAggregate.begin(), orientationAggregate.end());
        const double sumOri = std::accumulate(orientationAggregate.begin(), orientationAggregate.end(), 0.0);
        mLastOrientationMean = static_cast<float>(sumOri / std::max<size_t>(1, orientationAggregate.size()));
    }
    else
    {
        mLastOrientationMin = mLastOrientationMax = mLastOrientationMean = 0.f;
    }
    // Count the unique features finally classified as density anomalies.
    if(mConfig.enable_local_statistics && mConfig.enable_density_stat)
    {
        mLastDensityHighCount = 0;
        for(float r : mDensityRisks)
            if(r > 0.f)
                mLastDensityHighCount++;
    }
    else
    {
        mLastDensityHighCount = 0;
    }
    // Number of descriptor anomalies.
    if(mConfig.enable_local_statistics && mConfig.enable_descriptor_stat)
    {
        mLastDescriptorHighCount = 0;
        for(float r : mDescriptorRisks)
            if(r > 0.f)
                mLastDescriptorHighCount++;
    }
    else
    {
        mLastDescriptorHighCount = 0;
    }
    // Number of orientation anomalies.
    if(mConfig.enable_local_statistics && mConfig.enable_orientation_stat)
    {
        mLastOrientationHighCount = 0;
        for(float r : mOrientationRisks)
            if(r > 0.f)
                mLastOrientationHighCount++;
    }
    else
    {
        mLastOrientationHighCount = 0;
    }
}

void PatchDefense::ComputeTemporalWeights(Frame &frame)
{
    const size_t N = frame.N;
    mTemporalWeights.assign(N, 1.f);
    for(size_t i=0; i<N; ++i)
    {
        const float raw = (i < mTemporalRaw.size()) ? mTemporalRaw[i] : 0.f;
        mTemporalWeights[i] = std::exp(-mConfig.eta_temporal * raw);
    }
}

void PatchDefense::CombineWeights(Frame &frame)
{
    int safeCount = 0;
    for(int i=0; i<frame.N; ++i)
    {
        float wMask = mConfig.enable_risk_mask ? (1.f - frame.mvPatchRisk[i]) : 1.f;
        if(mConfig.enable_risk_mask)
        {
            const float rScaled = Clamp(mConfig.k_risk_mask * frame.mvPatchRisk[i], 0.f, 1.f);
            wMask = 1.f - rScaled;
        }
        wMask = Clamp(wMask * mConfig.k_mask_weight, 0.f, 1.f);

        float wTemp = 1.f;
        if(mConfig.enable_temporal_consistency && i < static_cast<int>(mTemporalWeights.size()))
            wTemp = Clamp(mTemporalWeights[i] * mConfig.k_temp_weight, 0.f, 1.f);

        const float wm = std::max(0.f, mConfig.weight_mask);
        const float wt = std::max(0.f, mConfig.weight_temp);
        const float sumGlobalW = wm + wt;
        float wRaw;
        if(sumGlobalW > 0.f)
        {
            wRaw = 1.f;
            if(wm > 0.f)
                wRaw *= std::pow(Clamp(wMask, 0.f, 1.f), wm / sumGlobalW);
            if(wt > 0.f)
                wRaw *= std::pow(Clamp(wTemp, 0.f, 1.f), wt / sumGlobalW);
        }
        else
        {
            wRaw = wMask * wTemp;
        }
        float wFinal = std::max(wRaw, mConfig.w_min);
        frame.mvPatchWeights[i] = Clamp(wFinal, mConfig.w_min, 1.f);
        if(frame.mvPatchWeights[i] > mConfig.safe_weight_threshold)
        {
            safeCount++;
        }
    }
    mLastSafeCount = safeCount;
    mLastPointCount = frame.N;
    mLastSafeRatio = static_cast<float>(safeCount) / std::max(1, frame.N);
}

bool PatchDefense::GetRiskMask(cv::Mat &out) const
{
    if(!mbEnabled || mLastRiskMask.empty())
        return false;
    mLastRiskMask.copyTo(out);
    return true;
}

} // namespace ORB_SLAM2
