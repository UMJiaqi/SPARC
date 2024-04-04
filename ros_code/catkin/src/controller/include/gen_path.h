#ifndef GEN_PATH
#define GEN_PATH

#include "common.h"
#define CIRCLE_PATH 1
#define SQUARE_PATH 2

void gen_path(const DTYPE front_z0, const DTYPE front_x0, const DTYPE scale, const int type, 
                std::vector<std::vector<DTYPE>> &path);
#endif
