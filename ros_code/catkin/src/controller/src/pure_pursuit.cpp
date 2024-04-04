/*
 *   Pure Pursuit Controller
**/

#include <iostream>
#include <math.h>
#include "pure_pursuit.h"
#include "common.h"
#include "MPCR.h"
#include "sensor.h"
#include "gen_path.h"
#include "logger.h"
#include "communicator.h"

PPC::PPC(
    MPCR *MPCR_model,
    sensor *msensor,
    communicator *mcommunicator,
    std::vector<std::vector<DTYPE>> &path,
    const DTYPE Lh2,
    const int control_type) {

    printf("[LOG INFO] Initialize the controller\n");
    ppc_MPCR = MPCR_model;
    ppc_sensor = msensor;
    ppc_communicator = mcommunicator;
    ppc_path = path;
    ppc_Lh2 = Lh2;
    ppc_control_type = control_type;
}

void PPC::start(const int mode) {
    printf("[LOG INFO] Move!\n");
    if (ppc_MPCR->get_section_num() == 1) {
        _algo_for_1_section(mode);
    } else if (ppc_MPCR->get_section_num() == 2) {
        _algo_for_2_section(mode);
    } else {
        printf("[LOG INFO] Error: Do not support %d sections\n", ppc_MPCR->get_section_num());
    }

}

void PPC::_algo_for_1_section(const int mode) {
    DTYPE error = 1.0;
    DTYPE l_min_angle0 = ppc_MPCR->convert_l_to_angle(ppc_MPCR->get_min_l());
    DTYPE l_min_angle1 = ppc_MPCR->convert_l_to_angle(ppc_MPCR->get_min_l());
    DTYPE l_min_angle2 = ppc_MPCR->convert_l_to_angle(ppc_MPCR->get_min_l());
    double turning_angle = 0;
    printf("[LOGINFO] min_l:%lf\n", ppc_MPCR->get_min_l());
    logger ppc_logger;

    if (mode != 3) {
        // Foot 0 fixed, foot 1 released.
        ppc_communicator->control_sucker('f', 'r');
        // Cavity 0-1 contracts to the maximum.
        ppc_communicator->control_cavity(l_min_angle0, l_min_angle1, l_min_angle2);
       // Foot 0 released, foot 1 fixed.
        ppc_communicator->control_sucker('r', 'f');  
    }

    // Record the results.
    ppc_logger.add_path(ppc_path);
    ppc_logger.add_pose(ppc_MPCR->get_coord());
    ppc_logger.write_path_to_file();
    printf("[LOG INFO] foot0:[%.4lf,%.4lf], foot1:[%.4lf,%.4lf], theta:%.4lf\n", 
            ppc_MPCR->get_foot_z(0), ppc_MPCR->get_foot_x(0),
            ppc_MPCR->get_foot_z(1), ppc_MPCR->get_foot_x(1),
            ppc_MPCR->get_theta());

    int step_i = 1;
    int ind = 0;
    while (error > 3e-3) {
        DTYPE R, omega, l[3], angle[3], new_re_z, new_re_x, new_re_theta, new_theta, global_z, global_x;
        int dir;

        // Based on the position of foot 0 and the front view distance, find pgoal and calculate the turning radius, turning angle, and turning direction.
        find_pgoal(0, ind, R, omega, dir);
        printf("[LOG INFO] R:%.4lf, omega:%.4lf, dir:%d, ind:%d\n", R, omega, dir, ind);

        // Based on the turning radius, turning angle, and turning direction, obtain the control quantity l and the new front foot landing coordinates and deflection angle relative to the front foot.
        if(get_new_re_coord(R, omega, dir, l, new_re_z, new_re_x, new_re_theta) == -1) break;

        // Calculate the angle corresponding to l.
        ppc_MPCR->convert_l_to_angle(l, angle);

	    turning_angle += new_re_theta * 180 / PI;
        printf("[LOG INFO] Step:%d, new_re_z:%.4lf, new_re_x:%.4lf, new_re_theta:%.4lf, turning_angle:%.4lf,l:[%.4lf,%.4lf,%.4lf], angle:[%.1lf,%.1lf,%.1lf]\n", \
                step_i, new_re_z, new_re_x, new_re_theta * 180 / PI, turning_angle, l[0], l[1], l[2], angle[0], angle[1], angle[2]);

        // Complete one forward crawling motion cycle.
        if (mode != 3) {
            ppc_communicator->control_cavity(angle[0], angle[1], angle[2]);             // Move the fore foot to the target position.
            ppc_communicator->control_sucker('f', 'r');                                 // Front foot fixed, rear foot released.
            ppc_communicator->control_cavity(l_min_angle, l_min_angle, l_min_angle);    // The cavitys contract to the shortest.
            ppc_communicator->control_sucker('r', 'f');                                 // Rear foot fixed, fore foot released.           
        }

        // Obtain the new global coordinates of foot 0.
        if (ppc_control_type == CLOSED_LOOP_CONTROL) {
            // Closed-loop control, read optrack data to obtain the global coordinates and orientation of the front foot.
            ppc_sensor->get_fore_coord(global_z, global_x);
            ppc_sensor->get_global_theta(new_theta);
        } else {
            // Open-loop control, directly obtain the global coordinates and orientation of the front foot using theoretical calculation results.
            ppc_MPCR->get_global_fcoord(new_re_z, new_re_x, 0, global_z, global_x);
            new_theta = new_re_theta + ppc_MPCR->get_theta();
        }

        printf("[LOG INFO] global_z:%.4lf, global_x:%.4lf, new_theta:%.4lf\n", global_z, global_x, new_theta);

        // Update the coordinates of the fore foot, rear foot, and robot orientation.
        ppc_MPCR->update_theta(new_theta);
        ppc_MPCR->update_coord(0, global_z, global_x);
        ppc_MPCR->update_coord(1, global_z - ppc_MPCR->get_min_l() * std::cos(new_theta), 
                                global_x - ppc_MPCR->get_min_l() * std::sin(new_theta));

        // Update Coordinate Error.
        error = std::sqrt(square(global_z - ppc_path[ppc_path.size() - 1][0]) + square(global_x - ppc_path[ppc_path.size() - 1][1]));
        
        // Record message. 
        ppc_logger.add_l_angle(0, l, angle);
        /*if (mode != 3) {
            // 从optitrack中读数据并写入文件
            DTYPE _global_z, _global_x, _new_theta, _l_min;
            _l_min = ppc_MPCR->get_min_l();
            ppc_sensor->get_fore_coord(_global_z, _global_x);
            ppc_sensor->get_global_theta(_new_theta);
            ppc_logger.add_pose(_global_z, _global_x, \
                                _global_z - _l_min * std::cos(_new_theta), \
                                _global_x - _l_min * std::sin(_new_theta));   
        } else {*/
            ppc_logger.add_pose(ppc_MPCR->get_coord());
        //}

        printf("[LOG INFO] foot0:[%.4lf,%.4lf], foot1:[%.4lf,%.4lf], theta:%.4lf\n",
                ppc_MPCR->get_foot_z(0), ppc_MPCR->get_foot_x(0),
                ppc_MPCR->get_foot_z(1), ppc_MPCR->get_foot_x(1),
                ppc_MPCR->get_theta());
        step_i++;
	    ppc_logger.write_coord_to_file();
    }

    ppc_logger.write_path_to_file();
    ppc_logger.write_coord_to_file();
    ppc_logger.write_angle_to_file();
    ppc_logger.write_l_to_file();
    printf("[LOG INFO] write all data to file\n");
}

void PPC::_algo_for_2_section(const int mode) {
    printf("[LOG INFO] pure pursuit controller for 2 section\n");

    DTYPE error = 1.0;
    DTYPE l_min_angle = ppc_MPCR->convert_l_to_angle(ppc_MPCR->get_min_l());
    printf("[LOGINFO] min_l:%lf\n", ppc_MPCR->get_min_l());
    logger ppc_logger;

    // 0 Foot Release, 1 Foot Fixed, 2 Foot Release.
    ppc_communicator->control_sucker('r', 'f', 'r');
    // 0-1, 1-2 Both chambers contract to the shortest length.
    ppc_communicator->control_cavity(l_min_angle, l_min_angle, l_min_angle, l_min_angle, l_min_angle, l_min_angle);
    // 0 Release, 1 Release, 2 Fixed.
    ppc_communicator->control_sucker('r', 'r', 'f');  

    ppc_logger.add_path(ppc_path);
    ppc_logger.add_pose(ppc_MPCR->get_coord());

    printf("[LOG INFO] foot0:[%.4lf,%.4lf], foot1:[%.4lf,%.4lf], foot2:[%.4lf,%.4lf], theta:%.4lf\n",
            ppc_MPCR->get_foot_z(0), ppc_MPCR->get_foot_x(0),
            ppc_MPCR->get_foot_z(1), ppc_MPCR->get_foot_x(1),
            ppc_MPCR->get_foot_z(2), ppc_MPCR->get_foot_x(2),
            ppc_MPCR->get_theta());
    int count = 0;
    int cur_ind = 0;
    while (error > 1e-4) {
        DTYPE l[3], angle[3];
        DTYPE R, omega, new_re_z, new_re_x, new_re_theta, global_z, global_x, new_theta;
        int dir;

        /* Chamber 0-1 contracts to the maximum, Chamber 1-2 moves forward. */

        // Based on the position of Foot 1 and the forward distance, find pgoal and calculate the turning radius, turning angle, and turning direction.
        find_pgoal(1, cur_ind, R, omega, dir);
        // Based on the turning radius, turning angle, and turning direction, obtain the control quantity l and the new Foot 1 landing point coordinates and deviation angle relative to Foot 1.
        if(get_new_re_coord(R, omega, dir, l, new_re_z, new_re_x, new_re_theta) == -1) break;
        // Calculate the corresponding angle for l.
        ppc_MPCR->convert_l_to_angle(l, angle);
        // Complete the forward movement of Foot 1.
        ppc_communicator->control_cavity(-1, -1, -1, angle[0], angle[1], angle[2]); // Foot 1 moves to the target position.
        ppc_communicator->control_sucker('r', 'f', 'r');                            // Foot 0 release, Foot 1 fix, Foot 2 release.
        // Obtain the new global coordinates of Foot 1.
        // Obtain the new global coordinates of Foot 1.
        if (ppc_control_type == CLOSED_LOOP_CONTROL) {
            // Closed-loop control, read optrack data to obtain the global coordinates and orientation of Foot 1.
            ppc_sensor->get_fore_coord(global_z, global_x);
            ppc_sensor->get_global_theta(new_theta);
        } else {
            // Open-loop control, directly obtain the global coordinates and orientation of Foot 1 and the robot using theoretical calculation results.
            ppc_MPCR->get_global_fcoord(new_re_z, new_re_x, 1, global_z, global_x);
            new_theta = new_re_theta + ppc_MPCR->get_theta();
        }
        // Update the new positions and orientations of each foot.
        ppc_MPCR->update_theta(new_theta);
        ppc_MPCR->update_coord(1, global_z, global_x);                                      // Update the coordinates of Foot 1.
        ppc_MPCR->update_coord(0, global_z + ppc_MPCR->get_min_l() * std::cos(new_theta), 
                                global_x + ppc_MPCR->get_min_l() * std::sin(new_theta));    // Update the coordinates of Foot 0 (contract to the shortest).

        ppc_logger.add_l_angle(1, l, angle);
        
        error = std::sqrt(square(global_z - ppc_path[ppc_path.size() - 1][0]) + square(global_x - ppc_path[ppc_path.size() - 1][1]));
        if (error < 1e-4) break;

        /* Chamber 0-1 moves forward, Chamber 1-2 contracts to the maximum. */

        // Based on the position of Foot 0 and the forward distance, find pgoal and calculate the turning radius, turning angle, and turning direction.
        find_pgoal(0, cur_ind, R, omega, dir);
        // Based on the turning radius, turning angle, and turning direction, obtain the control quantity l and the new landing coordinates and deviation angle of Foot 1 relative to Foot 1.
        if(get_new_re_coord(R, omega, dir, l, new_re_z, new_re_x, new_re_theta) == -1) break;
        // Calculate the turning angle corresponding to l.
        ppc_MPCR->convert_l_to_angle(l, angle);
        // Complete the forward movement of Foot 0 and the contraction of Foot 2.
        ppc_communicator->control_cavity(angle[0], angle[1], angle[2], \
                                        l_min_angle, l_min_angle, l_min_angle);                 // Foot 0 moves to the target position, Foot 2 contracts to the maximum.
        ppc_communicator->control_sucker('f', 'r', 'r');                                        // Foot 0 fixed, Foot 1 released, Foot 2 released.
        ppc_communicator->control_cavity(l_min_angle, l_min_angle, l_min_angle, -1, -1, -1);    // Chamber 0-1 contracts to the shortest.

        // Obtain the new global coordinates of Foot 0.
        if (ppc_control_type == CLOSED_LOOP_CONTROL) {
            // Closed-loop control, read the optrack data to obtain the global coordinates and orientation of Foot 1.
            ppc_sensor->get_fore_coord(global_z, global_x);
            ppc_sensor->get_global_theta(new_theta);
        } else {
            // Open-loop control, directly obtain the global coordinates and orientation of Foot 0 and the robot using theoretical calculation results.
            ppc_MPCR->get_global_fcoord(new_re_z, new_re_x, 0, global_z, global_x);
            new_theta = new_re_theta + ppc_MPCR->get_theta();
        }
        // Update the new positions and orientations of each foot.
        ppc_MPCR->update_theta(new_theta);
        ppc_MPCR->update_coord(0, global_z, global_x);  // Update the coordinates of Foot 0.
        ppc_MPCR->update_coord(1, global_z - ppc_MPCR->get_min_l() * std::cos(new_theta), 
                                global_x - ppc_MPCR->get_min_l() * std::sin(new_theta));   // Update the coordinates of Foot 1 (contracted to the shortest).
        ppc_MPCR->update_coord(2, global_z - 2 * ppc_MPCR->get_min_l() * std::cos(new_theta),
                                global_x - 2 * ppc_MPCR->get_min_l() * std::sin(new_theta));   // Update the coordinates of Foot 2 (contracted to the shortest).

        ppc_communicator->control_sucker('r', 'r', 'f');                            // Foot 0 released, Foot 1 released, Foot 2 fixed.

        error = std::sqrt(square(global_z - ppc_path[ppc_path.size() - 1][0]) + square(global_x - ppc_path[ppc_path.size() - 1][1]));

        ppc_logger.add_l_angle(0, l, angle);
        ppc_logger.add_pose(ppc_MPCR->get_coord());
        printf("[LOG INFO] foot0:[%.4lf,%.4lf], foot1:[%.4lf,%.4lf], foot2:[%.4lf,%.4lf], theta:%.4lf\n",
                    ppc_MPCR->get_foot_z(0), ppc_MPCR->get_foot_x(0),
                    ppc_MPCR->get_foot_z(1), ppc_MPCR->get_foot_x(1),
                    ppc_MPCR->get_foot_z(2), ppc_MPCR->get_foot_x(2),
                    ppc_MPCR->get_theta());
        count ++;
        if (count > 200) {
            printf("[LOG INFO] break loop\n");
            break;      
        }
    }

    ppc_logger.write_path_to_file();
    ppc_logger.write_coord_to_file();
    ppc_logger.write_angle_to_file();
    ppc_logger.write_l_to_file();
    printf("[LOG INFO] write all data to file\n");
}

int PPC::get_cloest_ind(const int foot, const int cur_ind) {
    int ind = 0;
    DTYPE dis = DBL_MAX;

    int start_ind = cur_ind - ppc_path.size() / 8;
    int end_ind = cur_ind + ppc_path.size() / 8;
    if (start_ind < 0) start_ind = 0;
    if (end_ind > ppc_path.size()) end_ind = ppc_path.size();

    for (int i = start_ind; i < end_ind; ++i) {
        DTYPE z1 = ppc_path[i][0];
        DTYPE x1 = ppc_path[i][1];
        DTYPE L2 = square(z1 - ppc_MPCR->get_foot_z(foot)) + square(x1 - ppc_MPCR->get_foot_x(foot));
        if (L2 < dis) {
            dis = L2;
            ind = i;
        }
    }

    return ind;
}

void PPC::find_pgoal(const int foot, int &cur_ind, DTYPE &R, DTYPE &omega, int &dir) {
    bool flag = false;
    DTYPE z_goal, x_goal, re_z, re_x;
    DTYPE z = ppc_MPCR->get_foot_z(foot);
    DTYPE x = ppc_MPCR->get_foot_x(foot);
    int goal_ind = -1;
    int start_ind = cur_ind;
    int end_ind = cur_ind + ppc_path.size() / 6;
    if (start_ind < 0) start_ind = 0;
    if (end_ind > ppc_path.size()) end_ind = ppc_path.size();
    printf("%d,%d\n", start_ind, end_ind);
    for (int i = start_ind; i < end_ind; ++i) {
        DTYPE z1 = ppc_path[i][0];
        DTYPE x1 = ppc_path[i][1];
        DTYPE L2 = square(z1 - z) + square(x1 - x);
        if (L2 >= ppc_Lh2) {
            goal_ind = i;
            flag = true;
            break;
        }
    }

    if (!flag) {
        goal_ind = ppc_path.size() - 1;
    }

    z_goal = ppc_path[goal_ind][0];
    x_goal = ppc_path[goal_ind][1];
    ppc_MPCR->get_relative_fcoord(z_goal, x_goal, foot, re_z, re_x);

    if (std::abs(re_x) < LST) { // Walk in a straight line.
        R = re_z;
        omega = 0;
        dir = 0;
    } else {    // Walk in an arc.
        R = ((square(re_x) + square(re_z)) / std::abs(2 * re_x));
        omega = std::abs(std::atan(re_z / (R - std::abs(re_x))));
        omega = R < std::abs(re_x)? PI - omega : omega;
        dir = re_x < 0 ? -1 : 1;
    }
    cur_ind = goal_ind;
    printf("%d\n", goal_ind);
}


#define get_new_l_theta(new_re_x, new_re_z, l, new_re_theta)                                    \
    do {                                                                                        \
        DTYPE tmp_rho, tmp_phi;                                                                 \
        ppc_MPCR->kinematic_model(new_re_x, 0, new_re_z, l, tmp_rho, tmp_phi, new_re_theta);    \
    } while(0);                                                                                 \
    do {                                                                                        \
        DTYPE tmp_rho, tmp_phi, tmp_theta;                                                      \
        ppc_MPCR->kinematic_model(new_re_x, -0.005, new_re_z, l, tmp_rho, tmp_phi, tmp_theta);  \
    } while(0);

int PPC::get_new_re_coord(const DTYPE R, const DTYPE omega, const int dir,
                            DTYPE l[3], DTYPE &new_re_z, DTYPE &new_re_x, DTYPE &new_re_theta) {
    const int n = 100;  //The number of divisions for omega.
    int ind = -1;
    if (dir != 0) { // Move along the arc line.
        for (int i = 0; i < n; ++i) {
            auto cur_omega = i * omega / n;
            // Calculate the coordinates of the target point relative to the next foot at this angle.
            auto re_z = R * std::sin(cur_omega) + ppc_MPCR->get_min_l();
            auto re_x = dir * (R - R * std::cos(cur_omega));
            if (ppc_MPCR->in_workspace(re_z, re_x)) {
                ind = i;
            }
        }
        if (ind == -1) return -1;

        // Calculate the coordinates and turning angle of the target point relative to the front foot at this angle.
        DTYPE cur_omega = ind * omega / n;
        new_re_z = R * std::sin(cur_omega);
        new_re_x = dir * (R - R * std::cos(cur_omega));
        get_new_l_theta(new_re_x, new_re_z + ppc_MPCR->get_min_l(), l, new_re_theta);
    } else {    // Move along the straight line.
        for (int i = 0; i < n; ++i) {
            auto cur_dis = i * R / n;
            auto re_z = cur_dis + ppc_MPCR->get_min_l();
            auto re_x = 0;
            if (ppc_MPCR->in_workspace(re_z, re_x)) {
                ind = i;
            }
        }
        if (ind == -1) return -1;
        new_re_z = ind * R / n;
        new_re_x = 0;
        get_new_l_theta(new_re_x, new_re_z + ppc_MPCR->get_min_l(), l, new_re_theta);
        new_re_theta = 0;
        /*l[0] = new_re_z + ppc_MPCR->get_min_l();
        l[1] = new_re_z + ppc_MPCR->get_min_l();
        l[2] = new_re_z + ppc_MPCR->get_min_l();*/
    }
    return 1;
}
