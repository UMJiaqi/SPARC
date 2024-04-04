#ifndef PURE_PURSUIT
#define PURE_PURSUIT

#include "common.h"
#include "MPCR.h"
#include "sensor.h"
#include "communicator.h"

#define OPEN_LOOP_CONTROL   19984081
#define CLOSED_LOOP_CONTROL 19984082

class PPC{
    private:
        void find_pgoal(const int foot, DTYPE &R, DTYPE &omega, int &dir);
        int get_new_re_coord(const DTYPE R, const DTYPE omega, const int dir,
                            DTYPE l[3], DTYPE &new_re_z, DTYPE &new_re_x, DTYPE &new_re_theta);
        int get_cloest_ind(const int foot);
        DTYPE ppc_Lh2;                                  // Square of lookahead distance for pure pursuit algorithm[meter^2]
        std::vector<std::vector<DTYPE>> ppc_path;       // reference track(nx2)[meter]
        int ppc_path_n;                                 // number of path's point
        int ppc_control_type;                           // open or closed loop control
        MPCR *ppc_MPCR;                                 // pointer of MPCR model
        sensor *ppc_sensor;                             // pointer of sensor reader
        communicator *ppc_communicator;                               // pointer of arduino communicator
    public:
        PPC(MPCR *, sensor *,  communicator *, std::vector<std::vector<DTYPE>> &path, const DTYPE Lh2, const int control_type);
        void start(const int mode);
        void _algo_for_1_section(const int mode);
        void _algo_for_2_section(const int mode);
};

#endif
