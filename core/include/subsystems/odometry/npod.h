#include "../core/include/utils/math_util.h"
#include "../core/include/utils/geometry.h"

#include <array>
struct PodConfig
{
    double wheel_radius;
    pose_t pose;
};

struct Factors
{
    double inv_Fx;
    double inv_Fy;
    double inv_Frot;
};
double dot(const point_t a, const point_t b)
{
    return a.x * b.x + a.y * b.y;
}

const point_t e_x = {.x = 1.0, .y = 0.0};
const point_t e_y = {.x = 0.0, .y = 1.0};

template <size_t num_pods>
std::array<Factors, num_pods> calc_factors(std::array<PodConfig, num_pods> &cfgs)
{
    std::array<Factors, num_pods> facs;
    int i = 0;
    for (PodConfig cfg : cfgs)
    {
        point_t d = point_t{.x = std::cos(cfg.pose.rot), .y = std::sin(cfg.pose.rot)};
        double inv_Fx = cfg.wheel_radius * dot(d, e_x);
        double inv_Fy = cfg.wheel_radius * dot(d, e_y);
        point_t v = cfg.pose.get_point();
        double v_len = cfg.pose.get_point().dist(point_t{0, 0});
        point_t rotated_v = {.x = -v.y, .y = v.x};
        double inv_Frot = cfg.wheel_radius * dot(d, rotated_v) / (v_len * v_len);
        facs[i] = {inv_Fx, inv_Fy, inv_Frot};
        i++;
        printf("%.2f, %.2f, %.2f\n", inv_Fx, inv_Fy, inv_Frot);
        fflush(stdout);
    }
    return facs;
};

template <size_t num_pods>
class NPod
{
public:
    NPod<num_pods>::NPod(std::array<PodConfig, num_pods> cfgs) : cfgs(cfgs), factors(calc_factors<num_pods>(cfgs)) 
    {
        if (cfgs.size() != num_pods)
        {
            printf("Different number of pods than expected. Expected %d, got %d\n", num_pods, cfgs.size());
            fflush(stdout);
            exit(1);
        }
        // static_assert(cfgs.size() == num_pods);
    }

    const std::array<Factors, num_pods> factors;
    const std::array<PodConfig, num_pods> cfgs;
};