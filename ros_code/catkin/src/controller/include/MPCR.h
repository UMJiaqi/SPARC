/*
    This is a class of Mobile Parallel Continuum Robots(MPCR)
    author: hjzhang
*/
#ifndef MPCR_CLASS
#define MPCR_CLASS

#include "common.h"
#include "cavity.h"

class MPCR {
    private:
        cavity *MPCR_cavity;                        // pointer of cavity
        int MPCR_n_section;                         // number of section
        DTYPE MPCR_r;                               // Distribution radius of cavities[meter]
        std::vector<std::vector<DTYPE>> MPCR_coord; // Coordinates of foots
        DTYPE MPCR_theta;                           // Angle with the z-axis[rad]
    public:
        MPCR(cavity *mcavity, const int n_section, const DTYPE r, const DTYPE init_fore_coord[2], const DTYPE init_theta);
        ~MPCR(){};
        void update_coord(const int foot, const DTYPE z, const DTYPE x);
        void update_theta(const DTYPE theta) {MPCR_theta = theta;};
        void kinematic_model(const DTYPE x, const DTYPE y, const DTYPE z,
                            DTYPE l[3], DTYPE &rho, DTYPE &phi, DTYPE &theta);
        bool in_workspace(const DTYPE re_z, const DTYPE re_x);
        int get_section_num() {return MPCR_n_section;}
        void get_relative_fcoord(const DTYPE global_z, const DTYPE global_x, const int foot, DTYPE &re_z, DTYPE &re_x);
        void get_global_fcoord(const DTYPE re_z, const DTYPE re_x, const int foot, DTYPE &global_z, DTYPE &global_x);
        DTYPE get_min_l() {return MPCR_cavity->c_min_l;};
        DTYPE get_max_l() {return MPCR_cavity->c_max_l;};
        DTYPE get_foot_z(const int foot) {return MPCR_coord[foot][0];}
        DTYPE get_foot_x(const int foot) {return MPCR_coord[foot][1];}
        std::vector<std::vector<DTYPE>> get_coord() {return MPCR_coord;}
        DTYPE get_theta() {return MPCR_theta;};
        void convert_l_to_angle(const DTYPE l[3], DTYPE angle[3]) {MPCR_cavity->convert_l_to_angle(l, angle);}
        DTYPE convert_l_to_angle(const DTYPE l) {return MPCR_cavity->convert_l_to_angle(l);}
        void convert_angle_to_l(const DTYPE angle[3], DTYPE l[3]) {MPCR_cavity->convert_angle_to_l(angle, l);}
        DTYPE convert_angle_to_l(const DTYPE angle) {return MPCR_cavity->convert_angle_to_l(angle);}
};

inline MPCR::MPCR(
    cavity *mcavity, 
    const int n_section,
    const DTYPE r, 
    const DTYPE init_fore_coord[2], 
    const DTYPE init_theta) {
    
    printf("[LOG INFO] Initialize the MPCR\n");

    MPCR_cavity = mcavity;
    MPCR_n_section = n_section;
    MPCR_r = r;
    MPCR_theta = init_theta;

    // The initial state is to shrink everything to the minimum.
    int foot_num = MPCR_n_section + 1;
    DTYPE z0 = init_fore_coord[0];
    DTYPE x0 = init_fore_coord[1];
    for (int i = 0; i < foot_num; ++i) {
        MPCR_coord.push_back({z0, x0});
        z0 = z0 - MPCR_cavity->c_min_l * std::cos(MPCR_theta);
        x0 = x0 - MPCR_cavity->c_min_l * std::sin(MPCR_theta);
    }
}

inline void MPCR::update_coord(const int foot, const DTYPE z, const DTYPE x) {
    MPCR_coord[foot][0] = z;
    MPCR_coord[foot][1] = x;
}

/**
 * Segmented kinematic model of Parallel Continuum Robots 
 * @param x,y,z     target coordinate of forefoot relative to the hindfoot
 * @param l[3]      the lengths of cavities
 * @param rho       radius of PCR
 * @param phi,theta the angle of PCR
 * @return void 
 */
inline void MPCR::kinematic_model(const DTYPE x, const DTYPE y, const DTYPE z, 
                            DTYPE l[3], DTYPE &rho, DTYPE &phi, DTYPE &theta) {
    phi = 0;
    if (std::abs(x) < LST && std::abs(y) < LST) {
        l[0] = z;
        l[1] = z;
        l[2] = z;
        rho = INF;
        phi = PI / 2;
        theta = 0;
        return;
    }
    if (std::abs(x) < LST) {
        phi = PI / 2;
    } else {
        phi = std::atan(y / x);
    }
    rho = (x * x + y * y + z * z) / (2 * std::sqrt(x * x + y * y));
    theta = std::acos(1 - std::sqrt(x * x + y * y) / rho);
    for (int i = 0; i < 3; ++i) {
        l[i] = 2 * MPCR_cavity->c_n * std::sin(theta / (2 * MPCR_cavity->c_n)) * \
                (rho - MPCR_r * std::cos(2 * PI / 3 * i + PI / 2 - phi));
    }
    if (x < 0) {
        std::swap(l[1], l[2]);
        theta = -theta;
    }
}

/**
 * get relative forefoot coordinates
 * @param re_z,re_x              target coordinates relative to hindfoot
 * @return bool                  true: in the workspace, false: out of workspace
 */
inline bool MPCR::in_workspace(const DTYPE re_z, const DTYPE re_x) {
    DTYPE l[3];
    DTYPE rho, phi, theta;
    DTYPE min_l = MPCR_cavity->c_min_l;
    DTYPE max_l = MPCR_cavity->c_max_l;
    kinematic_model(re_x, 0.0, re_z, l, rho, phi, theta);
    if (l[0] >= min_l && l[1] >= min_l && l[2] >= min_l && \
        l[0] <= max_l && l[1] <= max_l && l[2] <= max_l) {
            return true;
    } else {
        return false;
    }
}

/**
 * get relative forefoot coordinates
 * @param global_z, global_x     global coordinates
 * @param re_z,re_x              coordinates relative to re_foot foot
 * @return void
 */
inline void MPCR::get_relative_fcoord(
    const DTYPE global_z,
    const DTYPE global_x,
    const int re_foot,
    DTYPE &re_z,
    DTYPE &re_x) {

    DTYPE dz = global_z - MPCR_coord[re_foot][0];
    DTYPE dx = global_x - MPCR_coord[re_foot][1];
    re_z = dz * std::cos(MPCR_theta) + dx * std::sin(MPCR_theta);
    re_x = -dz * std::sin(MPCR_theta) + dx * std::cos(MPCR_theta);
}

/**
 * get global coordinates by relative forefoot coordinates
 * @param re_z,re_x             coordinates relative re_foot
 * @param global_z, global_x    global coordinates
 * @return void
 */
inline void MPCR::get_global_fcoord(
    const DTYPE re_z,
    const DTYPE re_x,
    const int re_foot,
    DTYPE &global_z,
    DTYPE &global_x) {

    global_z = re_z * std::cos(-MPCR_theta) + re_x * std::sin(-MPCR_theta) + MPCR_coord[re_foot][0];
    global_x = -re_z * std::sin(-MPCR_theta) + re_x * std::cos(-MPCR_theta) + MPCR_coord[re_foot][1];
}

#endif
