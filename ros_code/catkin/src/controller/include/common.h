#ifndef COMMON
#define COMMON

#include <string.h>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <float.h>
#include <algorithm>
#include <stdio.h>

#define DTYPE double
#define PI (double)(3.14159265358979323846)
#define INF DBL_MAX
#define LST 1e-6
#define square(x) ((x) * (x))
#define PATH_FOLLOWING_MODE 1

template<typename T>
inline T** new_2d(int height, int width) {
    T **mat = new T *[height];
    T *mat_data = new T [height * width];
    for (int i = 0; i < height; ++i) {
        mat[i] = &mat_data[i * width];
    }
    return mat;
}

template<typename T>
inline void delete_2d(T **ptr) {
    delete[] ptr[0];
    delete[] ptr;
}

#endif
