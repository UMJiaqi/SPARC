#ifndef CHARACTERIZATION
#define CHARACTERIZATION

#include "sensor.h"
#include "communicator.h"
#include "common.h"
#include <unistd.h>
#include <math.h>

class characterization
{
private:
    sensor *chara_sensor;
    communicator *chara_comm;
    double get_cur_l();
public:
    characterization(sensor *msensor, communicator *mcommunicator) {
        chara_sensor = msensor;
        chara_comm = mcommunicator;
    }
    void start();
    ~characterization();
};

inline double characterization::get_cur_l() {
    double fore_foot_coord[2], hind_foot_coord[2];
    chara_sensor->get_fore_coord(fore_foot_coord[0], fore_foot_coord[1]);
    chara_sensor->get_hind_coord(hind_foot_coord[0], hind_foot_coord[1]);
    return std::sqrt(square(fore_foot_coord[0] - hind_foot_coord[0]) + square(fore_foot_coord[1] - hind_foot_coord[1]));
}

inline void characterization::start() {
    double start_angle = 20;
    double end_angle = 150;
    int num_of_point = 50;

    std::string filename = (std::string)("characterization.txt");
    FILE * fp;
    fp = fopen(filename.c_str(), "w");
    for (int i = 0; i < num_of_point; ++i) {
        double cur_angle = (end_angle - start_angle) * i / num_of_point + start_angle;
        chara_comm->control_cavity(cur_angle, -1, -1);
        auto l = get_cur_l();
        fprintf(fp, "%lf, %lf\n", cur_angle, l);
        printf("%lf, %lf\n", cur_angle, l);
    }
    for (int i = num_of_point; i > 0; --i) {
        double cur_angle = (end_angle - start_angle) * i / num_of_point + start_angle;
        chara_comm->control_cavity(cur_angle, -1, -1);
        auto l = get_cur_l();
        fprintf(fp, "%lf, %lf\n", cur_angle, l);
        printf("%lf, %lf\n", cur_angle, l);
    }
    fclose(fp);
}

#endif
