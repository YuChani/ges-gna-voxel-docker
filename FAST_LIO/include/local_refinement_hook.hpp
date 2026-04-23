#ifndef LOCAL_REFINEMENT_HOOK_HPP
#define LOCAL_REFINEMENT_HOOK_HPP

#include <algorithm>
#include <deque>
#include <vector>
#include "common_lib.h"

struct LocalRefinementConfig
{
    bool enable = false;
    int recent_keyframe_limit = 10;
    int local_window_size = 5;
    double translation_step_size = 0.1;
};

struct RecentKeyframeState
{
    double lidar_end_time = 0.0;
    V3D position = Zero3d;
    M3D rotation = Eye3d;
    int downsampled_point_count = 0;
    int effective_correspondence_count = 0;
    double residual_mean = 0.0;
    double surface_score_mean = 1.0;
    double weighted_residual_mean = 0.0;
};

struct LocalRefinementEvaluation
{
    bool triggered = false;
    int window_size = 0;
    double window_residual_mean = 0.0;
    double latest_residual_mean = 0.0;
    double window_surface_score_mean = 1.0;
    double latest_surface_score_mean = 1.0;
    double window_weighted_residual_mean = 0.0;
    int window_correspondence_mean = 0;
};

struct LocalRefinementUpdate
{
    bool applied = false;
    double proxy_error_before = 0.0;
    double proxy_error_after = 0.0;
    V3D refined_position = Zero3d;
};

struct LocalRefinementResidualCheck
{
    bool valid = false;
    double proxy_error = 0.0;
};

class LocalRefinementHook
{
  public:
    void Configure(const LocalRefinementConfig &config)
    {
        config_ = config;
        TrimBuffer();
    }

    void PushKeyframe(const RecentKeyframeState &keyframe)
    {
        if (!config_.enable)
        {
            return;
        }
        keyframes_.push_back(keyframe);
        TrimBuffer();
    }

    bool HasSufficientWindow() const
    {
        return config_.enable && static_cast<int>(keyframes_.size()) >= config_.local_window_size;
    }

    std::vector<RecentKeyframeState> GetRecentWindow() const
    {
        const int take_count = std::min<int>(config_.local_window_size, keyframes_.size());
        return std::vector<RecentKeyframeState>(keyframes_.end() - take_count, keyframes_.end());
    }

    LocalRefinementEvaluation ReevaluateRecentWindow() const
    {
        LocalRefinementEvaluation eval;
        if (!HasSufficientWindow())
        {
            return eval;
        }

        const std::vector<RecentKeyframeState> window = GetRecentWindow();
        eval.triggered = true;
        eval.window_size = static_cast<int>(window.size());
        eval.latest_residual_mean = window.back().residual_mean;
        eval.latest_surface_score_mean = window.back().surface_score_mean;

        double residual_sum = 0.0;
        double surface_score_sum = 0.0;
        double weighted_residual_sum = 0.0;
        int correspondence_sum = 0;
        for (const auto &frame : window)
        {
            residual_sum += frame.residual_mean;
            surface_score_sum += frame.surface_score_mean;
            weighted_residual_sum += frame.weighted_residual_mean;
            correspondence_sum += frame.effective_correspondence_count;
        }

        eval.window_residual_mean = residual_sum / window.size();
        eval.window_surface_score_mean = surface_score_sum / window.size();
        eval.window_weighted_residual_mean = weighted_residual_sum / window.size();
        eval.window_correspondence_mean = correspondence_sum / static_cast<int>(window.size());
        return eval;
    }

    LocalRefinementUpdate RefineLatestKeyframePose(const RecentKeyframeState &latest_keyframe) const
    {
        LocalRefinementUpdate update;
        if (!HasSufficientWindow())
        {
            return update;
        }

        const std::vector<RecentKeyframeState> window = GetRecentWindow();
        if (window.size() < 2)
        {
            return update;
        }

        V3D anchor_mean = Zero3d;
        for (size_t i = 0; i + 1 < window.size(); ++i)
        {
            anchor_mean += window[i].position;
        }
        anchor_mean /= static_cast<double>(window.size() - 1);

        update.proxy_error_before = (latest_keyframe.position - anchor_mean).norm();
        update.refined_position = latest_keyframe.position + config_.translation_step_size * (anchor_mean - latest_keyframe.position);
        update.proxy_error_after = (update.refined_position - anchor_mean).norm();
        update.applied = update.proxy_error_after < update.proxy_error_before;
        return update;
    }

    void UpdateLatestKeyframe(const RecentKeyframeState &keyframe)
    {
        if (!config_.enable || keyframes_.empty())
        {
            return;
        }
        keyframes_.back() = keyframe;
    }

    LocalRefinementResidualCheck ReevaluateLatestProxyError(const RecentKeyframeState &latest_keyframe) const
    {
        LocalRefinementResidualCheck check;
        if (!HasSufficientWindow())
        {
            return check;
        }

        const std::vector<RecentKeyframeState> window = GetRecentWindow();
        if (window.size() < 2)
        {
            return check;
        }

        V3D anchor_mean = Zero3d;
        for (size_t i = 0; i + 1 < window.size(); ++i)
        {
            anchor_mean += window[i].position;
        }
        anchor_mean /= static_cast<double>(window.size() - 1);
        check.valid = true;
        check.proxy_error = (latest_keyframe.position - anchor_mean).norm();
        return check;
    }

  private:
    void TrimBuffer()
    {
        while (static_cast<int>(keyframes_.size()) > config_.recent_keyframe_limit)
        {
            keyframes_.pop_front();
        }
    }

    LocalRefinementConfig config_;
    std::deque<RecentKeyframeState> keyframes_;
};

#endif
