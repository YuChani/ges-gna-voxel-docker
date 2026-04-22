#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "primitive_residual_models.hpp"

using primitive_residual::PrimitiveResidualConfig;
using primitive_residual::PrimitiveResidualResult;

namespace
{

PointType MakePoint(const double x, const double y, const double z)
{
    PointType point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 0.0;
    point.curvature = 0.0;
    return point;
}

double SquaredDistance(const PointType &lhs, const PointType &rhs)
{
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
}

PointVector ExtractNearestPoints(const PointVector &points, const PointType &query, const int neighbor_count, const bool exclude_exact_match)
{
    std::vector<std::pair<double, int>> ranked_points;
    ranked_points.reserve(points.size());
    for (int i = 0; i < static_cast<int>(points.size()); ++i)
    {
        ranked_points.push_back(std::make_pair(SquaredDistance(points[i], query), i));
    }
    std::partial_sort(ranked_points.begin(),
                      ranked_points.begin() + std::min<int>(neighbor_count, ranked_points.size()),
                      ranked_points.end(),
                      [](const std::pair<double, int> &lhs, const std::pair<double, int> &rhs)
                      {
                          return lhs.first < rhs.first;
                      });

    PointVector local_points;
    const int take_count = std::min<int>(neighbor_count, ranked_points.size());
    local_points.reserve(take_count);
    for (int i = 0; i < take_count; ++i)
    {
        if (exclude_exact_match && ranked_points[i].first < 1e-12)
        {
            continue;
        }
        local_points.push_back(points[ranked_points[i].second]);
        if (static_cast<int>(local_points.size()) >= neighbor_count)
        {
            break;
        }
    }
    return local_points;
}

PointVector MakePlanePatch(const int count, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> xy_dist(-1.0, 1.0);
    std::normal_distribution<double> z_noise(0.0, 0.01);
    PointVector points;
    points.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        points.push_back(MakePoint(xy_dist(rng), xy_dist(rng), z_noise(rng)));
    }
    return points;
}

PointVector MakeEdgePatch(const int count, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> x_dist(0.0, 1.0);
    std::uniform_real_distribution<double> y_dist(-1.0, 1.0);
    std::normal_distribution<double> z_noise(0.0, 0.01);
    PointVector points;
    points.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        points.push_back(MakePoint(x_dist(rng), y_dist(rng), z_noise(rng)));
    }
    return points;
}

PointVector MakeCornerPatch(const int count, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> axis_dist(0.0, 1.0);
    std::uniform_real_distribution<double> other_dist(-1.0, 1.0);
    std::normal_distribution<double> noise(0.0, 0.01);
    PointVector points;
    points.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        if (i % 2 == 0)
        {
            points.push_back(MakePoint(axis_dist(rng), other_dist(rng), noise(rng)));
        }
        else
        {
            points.push_back(MakePoint(noise(rng), other_dist(rng), axis_dist(rng)));
        }
    }
    return points;
}

double EvaluateMeanAbsoluteResidual(const PointVector &points, const PrimitiveResidualConfig &config)
{
    double sum_abs = 0.0;
    int valid_count = 0;
    for (const auto &point : points)
    {
        const PointVector local_points = ExtractNearestPoints(points, point, config.neighbor_count + 1, true);
        PrimitiveResidualResult result;
        V3D body_point(point.x, point.y, point.z);
        if (primitive_residual::ComputePrimitiveResidual(local_points, point, body_point, config, result))
        {
            sum_abs += std::fabs(result.residual);
            ++valid_count;
        }
    }
    return valid_count > 0 ? sum_abs / valid_count : std::numeric_limits<double>::quiet_NaN();
}

void EvaluateSensitivity(std::ofstream &csv,
                         const std::string &scenario,
                         const std::string &metric,
                         const PointVector &points,
                         const PointType &reference_point,
                         const V3D &body_reference,
                         const V3D &direction,
                         const std::vector<double> &deltas,
                         const PrimitiveResidualConfig &baseline_config,
                         const PrimitiveResidualConfig &surface_config)
{
    for (double delta : deltas)
    {
        PointType perturbed = reference_point;
        perturbed.x += direction(0) * delta;
        perturbed.y += direction(1) * delta;
        perturbed.z += direction(2) * delta;
        V3D body_point = body_reference + direction * delta;
        const PointVector local_points = ExtractNearestPoints(points, perturbed, baseline_config.neighbor_count, false);

        PrimitiveResidualResult baseline_result;
        PrimitiveResidualResult surface_result;
        const bool baseline_valid = primitive_residual::ComputePrimitiveResidual(local_points, perturbed, body_point, baseline_config, baseline_result);
        const bool surface_valid = primitive_residual::ComputePrimitiveResidual(local_points, perturbed, body_point, surface_config, surface_result);

        csv << scenario << ',' << metric << ',' << std::fixed << std::setprecision(6) << delta << ','
            << (baseline_valid ? std::fabs(baseline_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
            << (surface_valid ? std::fabs(surface_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
            << (baseline_valid ? baseline_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << ','
            << (surface_valid ? surface_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << '\n';
    }
}

PointType RotatePointAroundAxis(const PointType &point,
                                const V3D &center,
                                const V3D &axis,
                                const double angle_rad)
{
    Eigen::AngleAxisd rotation(angle_rad, axis.normalized());
    V3D rotated = center + rotation * (V3D(point.x, point.y, point.z) - center);
    return MakePoint(rotated(0), rotated(1), rotated(2));
}

}

int main(int argc, char **argv)
{
    const std::string output_csv = argc > 1 ? argv[1] : std::string("primitive_benchmark.csv");
    std::mt19937 rng(42);

    PrimitiveResidualConfig plane_config;
    plane_config.mode = primitive_residual::PLANE_BASELINE;
    plane_config.neighbor_count = 5;
    plane_config.min_valid_neighbors = 5;

    PrimitiveResidualConfig center_config = plane_config;
    center_config.mode = primitive_residual::CENTER_GAUSSIAN;

    PrimitiveResidualConfig surface_config = plane_config;
    surface_config.mode = primitive_residual::SURFACE_LP;
    surface_config.lp_shape_p = 4.0;
    surface_config.lp_tangent_weight = 0.20;

    std::ofstream csv(output_csv, std::ios::out);
    csv << "scenario,metric,delta,plane_abs_residual,center_abs_residual,surface_abs_residual,plane_acceptance,center_acceptance,surface_acceptance\n";

    const std::vector<double> translation_deltas = {-0.30, -0.15, 0.0, 0.15, 0.30};
    const std::vector<double> rotation_deltas_deg = {-8.0, -4.0, 0.0, 4.0, 8.0};

    struct ScenarioSetup
    {
        std::string name;
        PointVector (*factory)(int, std::mt19937 &);
        PointType reference_point;
        V3D normal_direction;
        V3D tangent_direction;
    };

    const std::vector<ScenarioSetup> scenarios = {
        {"plane_patch", MakePlanePatch, MakePoint(0.2, -0.1, 0.0), V3D(0.0, 0.0, 1.0), V3D(1.0, 0.0, 0.0)},
        {"edge_patch", MakeEdgePatch, MakePoint(0.1, 0.0, 0.0), V3D(0.0, 0.0, 1.0), V3D(-1.0, 0.0, 0.0)},
        {"corner_patch", MakeCornerPatch, MakePoint(0.05, 0.0, 0.05), V3D(0.0, 0.0, 1.0), V3D(1.0, 0.0, 0.0)},
    };

    std::cout << "[Primitive Benchmark] output_csv=" << output_csv << std::endl;
    for (const auto &scenario : scenarios)
    {
        const PointVector points = scenario.factory(64, rng);
        const V3D body_reference(scenario.reference_point.x, scenario.reference_point.y, scenario.reference_point.z);

        const double plane_fit_error = EvaluateMeanAbsoluteResidual(points, plane_config);
        const double center_fit_error = EvaluateMeanAbsoluteResidual(points, center_config);
        const double surface_fit_error = EvaluateMeanAbsoluteResidual(points, surface_config);

        std::cout << std::fixed << std::setprecision(6)
                  << "[Scenario] " << scenario.name
                  << " fit_mean_abs_residual plane=" << plane_fit_error
                  << " center=" << center_fit_error
                  << " surface_lp=" << surface_fit_error << std::endl;

        for (const auto &metric_name : {std::make_pair(std::string("normal_translation"), scenario.normal_direction),
                                        std::make_pair(std::string("tangent_translation"), scenario.tangent_direction)})
        {
            for (double delta : translation_deltas)
            {
                PointType perturbed = scenario.reference_point;
                perturbed.x += metric_name.second(0) * delta;
                perturbed.y += metric_name.second(1) * delta;
                perturbed.z += metric_name.second(2) * delta;
                V3D body_point = body_reference + metric_name.second * delta;
                const PointVector local_points = ExtractNearestPoints(points, perturbed, plane_config.neighbor_count, false);

                PrimitiveResidualResult plane_result, center_result, surface_result;
                const bool plane_valid = primitive_residual::ComputePrimitiveResidual(local_points, perturbed, body_point, plane_config, plane_result);
                const bool center_valid = primitive_residual::ComputePrimitiveResidual(local_points, perturbed, body_point, center_config, center_result);
                const bool surface_valid = primitive_residual::ComputePrimitiveResidual(local_points, perturbed, body_point, surface_config, surface_result);

                csv << scenario.name << ',' << metric_name.first << ',' << std::fixed << std::setprecision(6) << delta << ','
                    << (plane_valid ? std::fabs(plane_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
                    << (center_valid ? std::fabs(center_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
                    << (surface_valid ? std::fabs(surface_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
                    << (plane_valid ? plane_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << ','
                    << (center_valid ? center_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << ','
                    << (surface_valid ? surface_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << '\n';
            }
        }

        for (double angle_deg : rotation_deltas_deg)
        {
            const double angle_rad = angle_deg * PI_M / 180.0;
            PointType rotated = RotatePointAroundAxis(scenario.reference_point,
                                                      V3D(0.0, 0.0, 0.0),
                                                      scenario.tangent_direction,
                                                      angle_rad);
            V3D body_rotated(rotated.x, rotated.y, rotated.z);
            const PointVector local_points = ExtractNearestPoints(points, rotated, plane_config.neighbor_count, false);
            PrimitiveResidualResult plane_result, center_result, surface_result;
            const bool plane_valid = primitive_residual::ComputePrimitiveResidual(local_points, rotated, body_rotated, plane_config, plane_result);
            const bool center_valid = primitive_residual::ComputePrimitiveResidual(local_points, rotated, body_rotated, center_config, center_result);
            const bool surface_valid = primitive_residual::ComputePrimitiveResidual(local_points, rotated, body_rotated, surface_config, surface_result);
            csv << scenario.name << ',' << "query_rotation_proxy" << ',' << angle_deg << ','
                << (plane_valid ? std::fabs(plane_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
                << (center_valid ? std::fabs(center_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
                << (surface_valid ? std::fabs(surface_result.residual) : std::numeric_limits<double>::quiet_NaN()) << ','
                << (plane_valid ? plane_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << ','
                << (center_valid ? center_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << ','
                << (surface_valid ? surface_result.acceptance_score : std::numeric_limits<double>::quiet_NaN()) << '\n';
        }
    }

    std::cout << "[Primitive Benchmark] done" << std::endl;
    return 0;
}
