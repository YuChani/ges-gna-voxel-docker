#ifndef PRIMITIVE_RESIDUAL_MODELS_HPP
#define PRIMITIVE_RESIDUAL_MODELS_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include "common_lib.h"

namespace primitive_residual
{

enum PrimitiveMode
{
    PLANE_BASELINE = 0,
    SURFACE_LP = 1,
    CENTER_GAUSSIAN = 2,
    HYBRID_PLANE_SURFACE = 3,
};

struct PrimitiveResidualConfig
{
    int mode = PLANE_BASELINE;
    int neighbor_count = 5;
    int min_valid_neighbors = 5;
    double max_match_sq = 5.0;
    double plane_fit_threshold = 0.1;
    double acceptance_score_threshold = 0.9;
    double lp_shape_p = 4.0;
    double lp_tangent_weight = 0.2;
    double lp_tangent_scale = 1.0;
    double lp_normal_scale = 1.0;
    double min_eigenvalue = 1e-4;
    double min_normal_scale = 0.05;
    double min_tangent_scale = 0.10;
    int hybrid_neighbor_count = 8;
    int hybrid_min_valid_neighbors = 6;
    double hybrid_min_planarity = 0.85;
    double hybrid_min_tangent_ratio = 0.10;
    double hybrid_weight_min = 0.50;
    double hybrid_weight_max = 1.00;
};

struct PrimitiveResidualResult
{
    bool valid = false;
    bool used_surface_model = false;
    V3D center = Zero3d;
    M3D frame = Eye3d;
    V3D scales = V3D(1.0, 1.0, 1.0);
    V3D world_gradient = Zero3d;
    V3D world_normal = V3D(0.0, 0.0, 1.0);
    double residual = 0.0;
    double signed_distance = 0.0;
    double gating_signed_distance = 0.0;
    double plane_residual = 0.0;
    double ekf_residual = 0.0;
    double surface_normalized_residual = 0.0;
    double surface_score = 1.0;
    double measurement_weight = 1.0;
    double tangent_lp = 0.0;
    double planarity = 0.0;
    double support_tangent_ratio = 1.0;
    double acceptance_score = 0.0;
};

inline double ClampPositive(const double value, const double floor_value)
{
    return std::max(value, floor_value);
}

inline double SafePowAbs(const double value, const double power)
{
    return std::pow(std::fabs(value), power);
}

inline double SafeSign(const double value)
{
    if (value > 0.0) return 1.0;
    if (value < 0.0) return -1.0;
    return 0.0;
}

inline bool IsHybridMode(const int mode)
{
    return mode == HYBRID_PLANE_SURFACE;
}

inline double Clamp01(const double value)
{
    return std::max(0.0, std::min(1.0, value));
}

inline double ClampRange(const double value, const double low, const double high)
{
    return std::max(low, std::min(high, value));
}

inline double BaselineAcceptanceScore(const double raw_signed_distance, const V3D &point_body)
{
    const double body_norm = std::sqrt(std::max(point_body.norm(), 1e-6));
    return 1.0 - 0.9 * std::fabs(raw_signed_distance) / body_norm;
}

inline double TangentScaleRatio(const V3D &scales)
{
    if (!scales.allFinite() || scales(0) <= 1e-9)
    {
        return 0.0;
    }
    return scales(1) / scales(0);
}

inline bool ComputeNeighborhoodFrame(const PointVector &points,
                                     const PrimitiveResidualConfig &config,
                                     V3D &center,
                                     M3D &frame,
                                     V3D &scales,
                                     double &planarity)
{
    if (static_cast<int>(points.size()) < config.min_valid_neighbors)
    {
        return false;
    }

    center.setZero();
    for (const auto &point : points)
    {
        center += V3D(point.x, point.y, point.z);
    }
    center /= static_cast<double>(points.size());

    M3D covariance = M3D::Zero();
    for (const auto &point : points)
    {
        V3D diff(point.x - center(0), point.y - center(1), point.z - center(2));
        covariance += diff * diff.transpose();
    }
    covariance /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<M3D> solver(covariance);
    if (solver.info() != Eigen::Success)
    {
        return false;
    }

    const V3D eigenvalues = solver.eigenvalues();
    const M3D eigenvectors = solver.eigenvectors();
    if (!eigenvalues.allFinite() || !eigenvectors.allFinite())
    {
        return false;
    }

    frame.col(0) = eigenvectors.col(2).normalized();
    frame.col(1) = eigenvectors.col(1).normalized();
    frame.col(2) = eigenvectors.col(0).normalized();

    scales(0) = ClampPositive(config.lp_tangent_scale * std::sqrt(ClampPositive(eigenvalues(2), config.min_eigenvalue)), config.min_tangent_scale);
    scales(1) = ClampPositive(config.lp_tangent_scale * std::sqrt(ClampPositive(eigenvalues(1), config.min_eigenvalue)), config.min_tangent_scale);
    scales(2) = ClampPositive(config.lp_normal_scale * std::sqrt(ClampPositive(eigenvalues(0), config.min_eigenvalue)), config.min_normal_scale);

    const double eig_sum = std::max(eigenvalues.sum(), config.min_eigenvalue);
    planarity = 1.0 - eigenvalues(0) / eig_sum;
    return true;
}

inline bool ComputePlaneResidual(const PointVector &points,
                                 const PointType &point_world,
                                 const V3D &point_body,
                                 const PrimitiveResidualConfig &config,
                                 PrimitiveResidualResult &result)
{
    if (static_cast<int>(points.size()) < config.min_valid_neighbors)
    {
        return false;
    }

    MatrixXf A(points.size(), 3);
    MatrixXf b(points.size(), 1);
    b.setOnes();
    b *= -1.0f;
    for (int j = 0; j < static_cast<int>(points.size()); ++j)
    {
        A(j, 0) = points[j].x;
        A(j, 1) = points[j].y;
        A(j, 2) = points[j].z;
    }

    Vector3f normvec = A.colPivHouseholderQr().solve(b);
    const double norm = normvec.norm();
    if (!std::isfinite(norm) || norm < 1e-9)
    {
        return false;
    }

    Vector4f plane;
    plane(0) = normvec(0) / norm;
    plane(1) = normvec(1) / norm;
    plane(2) = normvec(2) / norm;
    plane(3) = 1.0f / norm;

    for (const auto &sample : points)
    {
        const double support_residual = plane(0) * sample.x + plane(1) * sample.y + plane(2) * sample.z + plane(3);
        if (std::fabs(support_residual) > config.plane_fit_threshold)
        {
            return false;
        }
    }

    V3D center;
    M3D frame;
    V3D scales;
    double planarity = 0.0;
    if (!ComputeNeighborhoodFrame(points, config, center, frame, scales, planarity))
    {
        return false;
    }

    result.valid = true;
    result.used_surface_model = false;
    result.center = center;
    result.frame = frame;
    result.scales = scales;
    result.world_normal = V3D(plane(0), plane(1), plane(2));
    result.world_gradient = result.world_normal;
    result.signed_distance = plane(0) * point_world.x + plane(1) * point_world.y + plane(2) * point_world.z + plane(3);
    result.gating_signed_distance = result.signed_distance;
    result.plane_residual = result.signed_distance;
    result.ekf_residual = result.plane_residual;
    result.surface_normalized_residual = 0.0;
    result.surface_score = 1.0;
    result.measurement_weight = 1.0;
    result.residual = result.signed_distance;
    result.tangent_lp = 0.0;
    result.planarity = planarity;
    result.support_tangent_ratio = TangentScaleRatio(scales);

    result.acceptance_score = BaselineAcceptanceScore(result.gating_signed_distance, point_body);
    return std::isfinite(result.acceptance_score);
}

inline bool ComputeSurfaceLpResidual(const PointVector &points,
                                     const PointType &point_world,
                                     const V3D &point_body,
                                     const PrimitiveResidualConfig &config,
                                     PrimitiveResidualResult &result)
{
    if (static_cast<int>(points.size()) < config.min_valid_neighbors)
    {
        return false;
    }

    if (!std::isfinite(config.lp_shape_p) || config.lp_shape_p < 1.1)
    {
        return false;
    }

    V3D center;
    M3D frame;
    V3D scales;
    double planarity = 0.0;
    if (!ComputeNeighborhoodFrame(points, config, center, frame, scales, planarity))
    {
        return false;
    }

    V3D diff(point_world.x - center(0), point_world.y - center(1), point_world.z - center(2));
    const V3D local = frame.transpose() * diff;

    const double tangent_u = local(0) / scales(0);
    const double tangent_v = local(1) / scales(1);
    const double tangent_sum = SafePowAbs(tangent_u, config.lp_shape_p) + SafePowAbs(tangent_v, config.lp_shape_p);
    const double tangent_lp = std::pow(std::max(tangent_sum, 0.0), 1.0 / config.lp_shape_p);
    const double tangent_gate = 1.0 + config.lp_tangent_weight * tangent_lp;
    const double normal_scale = scales(2);

    const double signed_distance = local(2);
    const double residual = signed_distance / (normal_scale * tangent_gate);

    V3D d_tangent_lp_world = Zero3d;
    if (tangent_sum > 1e-12)
    {
        const double tangent_base = std::pow(tangent_sum, 1.0 / config.lp_shape_p - 1.0);
        const double grad_u = tangent_base * SafePowAbs(tangent_u, config.lp_shape_p - 1.0) * SafeSign(tangent_u) / scales(0);
        const double grad_v = tangent_base * SafePowAbs(tangent_v, config.lp_shape_p - 1.0) * SafeSign(tangent_v) / scales(1);
        d_tangent_lp_world = frame.col(0) * grad_u + frame.col(1) * grad_v;
    }

    const V3D normal_grad = frame.col(2) / (normal_scale * tangent_gate);
    const V3D tangent_grad = signed_distance * config.lp_tangent_weight * d_tangent_lp_world /
                             (normal_scale * tangent_gate * tangent_gate);

    result.valid = true;
    result.used_surface_model = true;
    result.center = center;
    result.frame = frame;
    result.scales = scales;
    result.world_normal = frame.col(2);
    result.world_gradient = normal_grad - tangent_grad;
    result.signed_distance = signed_distance;
    result.gating_signed_distance = signed_distance;
    result.plane_residual = signed_distance;
    result.ekf_residual = residual;
    result.surface_normalized_residual = residual;
    result.residual = residual;
    result.tangent_lp = tangent_lp;
    result.planarity = planarity;
    result.support_tangent_ratio = TangentScaleRatio(scales);

    result.acceptance_score = BaselineAcceptanceScore(result.residual, point_body);
    result.surface_score = Clamp01(result.acceptance_score);
    result.measurement_weight = 1.0;
    return result.world_gradient.allFinite() && std::isfinite(result.acceptance_score);
}

inline bool ComputeCenterGaussianResidual(const PointVector &points,
                                          const PointType &point_world,
                                          const V3D &point_body,
                                          const PrimitiveResidualConfig &config,
                                          PrimitiveResidualResult &result)
{
    if (static_cast<int>(points.size()) < config.min_valid_neighbors)
    {
        return false;
    }

    V3D center;
    M3D frame;
    V3D scales;
    double planarity = 0.0;
    if (!ComputeNeighborhoodFrame(points, config, center, frame, scales, planarity))
    {
        return false;
    }

    V3D diff(point_world.x - center(0), point_world.y - center(1), point_world.z - center(2));
    const V3D local = frame.transpose() * diff;
    const double u = local(0) / scales(0);
    const double v = local(1) / scales(1);
    const double w = local(2) / scales(2);
    const double ellipsoid_norm = std::sqrt(std::max(u * u + v * v + w * w, 1e-12));
    const double residual = ellipsoid_norm - 1.0;

    V3D local_gradient(u / (scales(0) * ellipsoid_norm),
                       v / (scales(1) * ellipsoid_norm),
                       w / (scales(2) * ellipsoid_norm));

    result.valid = true;
    result.used_surface_model = false;
    result.center = center;
    result.frame = frame;
    result.scales = scales;
    result.world_normal = frame.col(2);
    result.world_gradient = frame * local_gradient;
    result.signed_distance = diff.norm();
    result.gating_signed_distance = result.signed_distance;
    result.plane_residual = residual;
    result.ekf_residual = residual;
    result.surface_normalized_residual = residual;
    result.residual = residual;
    result.tangent_lp = std::sqrt(u * u + v * v);
    result.planarity = planarity;
    result.support_tangent_ratio = TangentScaleRatio(scales);

    result.acceptance_score = BaselineAcceptanceScore(result.residual, point_body);
    result.surface_score = Clamp01(result.acceptance_score);
    result.measurement_weight = 1.0;
    return result.world_gradient.allFinite() && std::isfinite(result.acceptance_score);
}

inline bool PassHybridSupportQuality(const PrimitiveResidualConfig &config,
                                     const PrimitiveResidualResult &surface_result)
{
    if (!surface_result.scales.allFinite())
    {
        return false;
    }
    if (surface_result.planarity < config.hybrid_min_planarity)
    {
        return false;
    }
    if (surface_result.scales(0) <= 0.0 || surface_result.scales(1) <= 0.0 || surface_result.scales(2) <= 0.0)
    {
        return false;
    }
    if (surface_result.support_tangent_ratio < config.hybrid_min_tangent_ratio)
    {
        return false;
    }
    return true;
}

inline double ComputeHybridSurfaceScore(const PrimitiveResidualConfig &config,
                                        const PrimitiveResidualResult &surface_result,
                                        const V3D &point_body)
{
    const double residual_score = Clamp01(BaselineAcceptanceScore(surface_result.surface_normalized_residual, point_body));
    const double planarity_den = std::max(1e-6, 1.0 - config.hybrid_min_planarity);
    const double planarity_score = Clamp01((surface_result.planarity - config.hybrid_min_planarity) / planarity_den);
    const double tangent_score = 1.0 / (1.0 + std::max(0.0, config.lp_tangent_weight) * std::max(0.0, surface_result.tangent_lp));
    return Clamp01(0.60 * residual_score + 0.25 * planarity_score + 0.15 * tangent_score);
}

inline bool ComputeHybridPlaneSurfaceResidual(const PointVector &points,
                                             const PointType &point_world,
                                             const V3D &point_body,
                                             const PrimitiveResidualConfig &config,
                                             PrimitiveResidualResult &result)
{
    PrimitiveResidualResult plane_result;
    PrimitiveResidualResult surface_result;
    if (!ComputePlaneResidual(points, point_world, point_body, config, plane_result))
    {
        return false;
    }
    if (!ComputeSurfaceLpResidual(points, point_world, point_body, config, surface_result))
    {
        return false;
    }
    if (!PassHybridSupportQuality(config, surface_result))
    {
        return false;
    }

    result = plane_result;
    result.used_surface_model = true;
    result.world_gradient = plane_result.world_normal;
    result.world_normal = plane_result.world_normal;
    result.gating_signed_distance = plane_result.signed_distance;
    result.signed_distance = plane_result.signed_distance;
    result.plane_residual = plane_result.residual;
    result.ekf_residual = plane_result.residual;
    result.residual = result.ekf_residual;
    result.surface_normalized_residual = surface_result.residual;
    result.surface_score = ComputeHybridSurfaceScore(config, surface_result, point_body);
    result.measurement_weight = ClampRange(config.hybrid_weight_min +
                                           (config.hybrid_weight_max - config.hybrid_weight_min) * result.surface_score,
                                           std::min(config.hybrid_weight_min, config.hybrid_weight_max),
                                           std::max(config.hybrid_weight_min, config.hybrid_weight_max));
    result.tangent_lp = surface_result.tangent_lp;
    result.planarity = surface_result.planarity;
    result.scales = surface_result.scales;
    result.frame = surface_result.frame;
    result.center = surface_result.center;
    result.support_tangent_ratio = surface_result.support_tangent_ratio;
    result.acceptance_score = plane_result.acceptance_score;
    return result.world_gradient.allFinite() &&
           std::isfinite(result.acceptance_score) &&
           std::isfinite(result.surface_score) &&
           std::isfinite(result.measurement_weight);
}

inline bool ComputePrimitiveResidual(const PointVector &points,
                                     const PointType &point_world,
                                     const V3D &point_body,
                                     const PrimitiveResidualConfig &config,
                                     PrimitiveResidualResult &result)
{
    if (config.mode == HYBRID_PLANE_SURFACE)
    {
        return ComputeHybridPlaneSurfaceResidual(points, point_world, point_body, config, result);
    }
    if (config.mode == SURFACE_LP)
    {
        return ComputeSurfaceLpResidual(points, point_world, point_body, config, result);
    }
    if (config.mode == CENTER_GAUSSIAN)
    {
        return ComputeCenterGaussianResidual(points, point_world, point_body, config, result);
    }
    return ComputePlaneResidual(points, point_world, point_body, config, result);
}

inline const char *ModeName(const int mode)
{
    if (mode == HYBRID_PLANE_SURFACE) return "hybrid_plane_surface";
    if (mode == SURFACE_LP) return "surface_lp";
    if (mode == CENTER_GAUSSIAN) return "center_gaussian";
    return "plane_baseline";
}

}

#endif
