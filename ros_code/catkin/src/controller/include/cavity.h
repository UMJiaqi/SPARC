#ifndef CAVITY
#define CAVITY

#include "common.h"

class cavity {
    private:
        DTYPE c_a;                      // Side length of the bottom polygon
        DTYPE c_A0;                     // characteriaztion parameter
        DTYPE c_B0;                     // characteriaztion parameter
        DTYPE c_C0;                     // characteriaztion parameter
    public:
        int c_n;                        // Number of layers
        DTYPE c_min_l;                  // Minimum length of the cavity[meter]
        DTYPE c_max_l;                  // Maximum length of the cavity[meter]
        cavity(int n, DTYPE min_l, DTYPE max_l, DTYPE a, DTYPE A0, DTYPE B0, DTYPE C0) : \
                c_n(n), c_min_l(min_l), c_max_l(max_l), c_a(a), c_A0(A0), c_B0(B0), c_C0(C0) {printf("[LOG INFO] Initialize the cavity\n");}
        ~cavity(){}
        void convert_l_to_angle(const DTYPE l[3], DTYPE angle[3]);
        DTYPE convert_l_to_angle(const DTYPE l);
        void convert_angle_to_l(const DTYPE angle[3], DTYPE l[3]);
        DTYPE convert_angle_to_l(const DTYPE angle);
};

/**
 * convert length of cavity to rotation angle
 * @param l                      length of cavities[m]
 * @param angle                  rotation angle of cavities[angle]
 */
inline void cavity::convert_l_to_angle(const DTYPE l[3], DTYPE angle[3]) {
    for (int i = 0; i < 3; ++i) {
        angle[i] = convert_l_to_angle(l[i]);
    }
}

inline DTYPE cavity::convert_l_to_angle(const DTYPE l) {
    auto h = l / c_n * 1000;    //mm
    DTYPE error = 1;
    DTYPE x0 = 0;               //angle
    // Netwon Method: x1 = x0 - f(x0)/df(x0)
    int count = 0;
    int max_iter = 500;
    while (error > LST && count < max_iter) {
        auto f = square(c_A0 * std::exp(-c_B0 * x0) + c_C0) + 2 * c_a * c_a * (std::cos(x0 / 180 * PI) - 1) - h * h;
        auto df = -2 * square(c_A0) * c_B0 * square(std::exp(-2 * c_B0 * x0)) - \
                2 * c_A0 * c_B0 * c_C0 * std::exp(-c_B0 * x0) - 2 * c_a * c_a * std::sin(x0 / 180 * PI);
        x0 = x0 - f / df;
        error = std::abs(square(c_A0 * std::exp(-c_B0 * x0) + c_C0) + 2 * c_a * c_a * (std::cos(x0 / 180 * PI) - 1) - h * h);
        count ++;
    }
    if (error > 1e-3) {
        printf("error! When l = %lf, can't calculate its angle\n", l);
        exit(-1);
    }
    auto angle = x0 * c_n;
    return angle;
}

/**
 * convert rotation angle of cavity to length
 * @param angle                  rotation angle of cavities
 * @param l                      length of cavities[m]
 */
inline void cavity::convert_angle_to_l(const DTYPE angle[3], DTYPE l[3]) {
    for (int i = 0; i < 3; ++i) {
        l[i] = convert_angle_to_l(angle[i]);
    }
}

inline DTYPE cavity::convert_angle_to_l(const DTYPE angle) {
    auto phi = angle / c_n;
    auto b = c_A0 * std::exp(-c_B0 * phi) + c_C0;
    auto l = c_n * std::sqrt(b * b + 2 * c_a * c_a * (std::cos(phi) - 1));
    return l;
}


#endif
