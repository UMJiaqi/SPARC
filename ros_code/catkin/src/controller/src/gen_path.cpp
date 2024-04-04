#include "gen_path.h"
#include <math.h>

#define CIRCLE_R (double(0.15))
#define SQUARE_L (double(0.3))
#define SIN_MAX_Z (double(0.4))
#define SIN_MAX_X (double(0.1))
#define STRAIGHT_MAX_Z (double(0.7))

#define get_quantile_val(i, n, a, b) ((a) + (i) * ((b) - (a)) / (n))

void gen_square(
    const DTYPE l,
    const DTYPE center_z,
    const DTYPE center_x,
    const DTYPE theta_start,
    const DTYPE theta_end,
    const int n,
    std::vector<std::vector<DTYPE>> &path)
{
    for (int i = 0; i < n; ++i) {
        DTYPE cur_theta = get_quantile_val(i, n, theta_start, theta_end);

        if (cur_theta < 0) cur_theta += 2 * PI;
        if (cur_theta > 2 * PI) cur_theta -= 2 * PI;
        DTYPE z, x;
        if (cur_theta >= 5 * PI / 4 && cur_theta <= 7 * PI / 4) {
            z = get_quantile_val(cur_theta - 5 * PI / 4, PI / 2, -l / 2, l / 2);
            x = -l / 2;
        } else if (cur_theta >= 7 * PI / 4 || cur_theta <= 1 * PI / 4) {
            z = l / 2;
            if (cur_theta >= 7 * PI / 4) {
                x = get_quantile_val(cur_theta - 7 * PI / 4, PI / 2, -l / 2, l / 2);
            } else {
                x = get_quantile_val(cur_theta - 0, PI / 4, 0, l / 2);
            }
        } else if (cur_theta >= 1 * PI / 4 && cur_theta <= 3 * PI / 4) {
            z = get_quantile_val(cur_theta - 1 * PI / 4, PI / 2, l / 2, -l / 2);
            x = l / 2;
        } else if (cur_theta >= 3 * PI / 4 && cur_theta <= 5 * PI / 4) {
            z = -l / 2;
            x = get_quantile_val(cur_theta - 3 * PI / 4, PI / 2, l / 2, -l / 2);
        }
        z += center_z;
        x += center_x;
        path.push_back({z, x});
    }
}

/**
 * Generate reference path
 * @param front_z0,front_x0     Forefoot initial coordinates
 * @param type                  if type==1 circular path, if type==2 square path
 * @param ppc_path              reference path coordinates
 * @return void 
 */
void gen_path(const DTYPE front_z0, const DTYPE front_x0, const DTYPE scale, const int type, 
                std::vector<std::vector<DTYPE>> &path) {
    printf("[LOG INFO] Generate reference path\n");
    if (type == CIRCLE_PATH) {
        auto r = CIRCLE_R;
        int path_n = 1000;  // 1000 points
        for (int i = 0; i < path_n; ++i) {
            auto cur_angle = i * 2 * PI / path_n - 1 * PI / 2;
            auto z = r * std::cos(cur_angle) + front_z0;
            auto x = r + r * std::sin(cur_angle) + front_x0;
            path.push_back({z, x});
        }
    } else if (type == SQUARE_PATH) {
        DTYPE dl = 0.001;
        DTYPE l = SQUARE_L;
        int n = l / dl;
        int half_n = n / 2;
        int i = 0;
        for (; i < half_n; ++i) {
            auto z = front_z0 + l / 2 * i / half_n;
            auto x = front_x0;
            path.push_back({z, x});
        }

        for (; i < half_n + n; ++i) {
            auto z = front_z0 + l / 2;
            auto x = front_x0 + l * (i - half_n) / n;
            path.push_back({z, x});
        }

        for (; i < half_n + n + n; ++i) {
            auto z = front_z0 + l / 2 - l * (i - half_n - n) / n;
            auto x = front_x0 + l;
            path.push_back({z, x});
        }

        for (; i < half_n + n + n + n; ++i) {
            auto z = front_z0 - l / 2;
            auto x = l + front_x0 - l * (i - half_n - n - n) / n;
            path.push_back({z, x});
        }

        for (; i < half_n + n + n + n + half_n; ++i) {
            auto z = front_z0 - l / 2 + l / 2 * (i - half_n - n - n - n) / half_n;
            auto x = front_x0;
            path.push_back({z, x});
        }
    } else if (type == SIN_PATH) {
        int n = 1000;
        for (int i = 0; i < n; ++i) {
            auto z = i * SIN_MAX_Z / n;
            auto x = SIN_MAX_X * sin(z / SIN_MAX_Z * 2 * PI);
            path.push_back({z, x});
        }
	for (int i = 0; i < n; ++i) {
	    auto z = SIN_MAX_Z + i * (0.1) / n;
	    auto x = (double)0.0;
	    path.push_back({z, x});
	}
    } else if (type == STRAIGHT_PATH) {
    	int n = 4000;
        for (int i = 0; i < n; ++i) {
            auto z = i * STRAIGHT_MAX_Z / n;
            auto x = (double)0.0;
            path.push_back({z, x});
        }
    } else if (type == DOUBLE_CIRCLE_PATH) {
        int n = 4000 / 4;
        auto r = CIRCLE_R;
        {
            DTYPE center_z = 0.0;
            DTYPE center_x = CIRCLE_R;
            for (int i = 0; i < n; ++i) {
                auto cur_angle = get_quantile_val(i, n, 3 * PI / 2, PI / 2);
                auto z = r * std::cos(cur_angle) + center_z;
                auto x = r * std::sin(cur_angle) + center_x;
                path.push_back({z, x});
            }
        }
        {
            DTYPE center_z = 0.0;
            DTYPE center_x = CIRCLE_R * 3;
            for (int i = 0; i < n; ++i) {
                auto cur_angle = get_quantile_val(i, n, -PI / 2, PI / 2);
                auto z = r * std::cos(cur_angle) + center_z;
                auto x = r * std::sin(cur_angle) + center_x;
                path.push_back({z, x});
            }
        }
        /*{
            DTYPE center_z = 0.0;
            DTYPE center_x = CIRCLE_R * 3;
            for (int i = 0; i < n; ++i) {
                auto cur_angle = get_quantile_val(i, n, PI / 2, -PI / 2);
                auto z = r * std::cos(cur_angle) + center_z;
                auto x = r * std::sin(cur_angle) + center_x;
                path.push_back({z, x});
            }
        }
        {
            DTYPE center_z = 0.0;
            DTYPE center_x = CIRCLE_R;
            for (int i = 0; i < n; ++i) {
                auto cur_angle = get_quantile_val(i, n, PI / 2, 3 * PI / 2);
                auto z = r * std::cos(cur_angle) + center_z;
                auto x = r * std::sin(cur_angle) + center_x;
                path.push_back({z, x});
            }
        }*/
    } else if (type == DOUBLE_SQUARE_PATH) {
        int n = 4000 / 4;
        gen_square(SQUARE_L, -1 * SQUARE_L / 2, 1 * SQUARE_L / 2, 7 * PI / 4, PI / 4, n, path);
        gen_square(SQUARE_L, -1 * SQUARE_L / 2, 3 * SQUARE_L / 2, -1 * PI / 4, 3 * PI / 4, n, path);
        //gen_square(SQUARE_L, 0, 3 * SQUARE_L / 2, 1 * PI / 2, -PI / 2, n, path);
        //gen_square(SQUARE_L, 0, 1 * SQUARE_L / 2, 1 * PI / 2, 3 * PI / 2, n, path);
    }
}
