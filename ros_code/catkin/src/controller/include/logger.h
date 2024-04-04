#ifndef RECORD
#define RECORD

#include <vector>
#include <string>
#include <stdio.h>
#include "common.h"
#include "MPCR.h"

class logger {
    private:
        std::vector<std::vector<std::vector<DTYPE>>> logger_coord;
        std::vector<std::vector<DTYPE>> logger_path;
        std::vector<std::vector<std::vector<DTYPE>>> logger_l;
        std::vector<std::vector<std::vector<DTYPE>>> logger_angle;

    public:
        logger() {
            std::vector<std::vector<DTYPE>> tmp;
            logger_l.push_back(tmp);
            logger_l.push_back(tmp);
            logger_angle.push_back(tmp);
            logger_angle.push_back(tmp);
        };

        ~logger() {};

        void add_pose(std::vector<std::vector<DTYPE>> coord) {
            logger_coord.push_back(coord);
        }

        void add_path(const std::vector<std::vector<DTYPE>> &path) {
            logger_path = path;
        }

        void add_l_angle(const int cavity, const DTYPE l[3], const DTYPE angle[3]) {
            logger_l[cavity].push_back({l[0], l[1], l[2]});
            logger_angle[cavity].push_back({angle[0], angle[1], angle[2]});
        }

        void write_coord_to_file() {
            for (int foot = 0; foot < logger_coord[0].size(); ++foot) {
                for (int i = 0; i < 2; ++i) {
                    std::string filename;
                    if (i == 0) {
                        filename = (std::string)("foot_") + std::to_string(foot) + "_z.txt";
                    } else {
                        filename = (std::string)("foot_") + std::to_string(foot) + "_x.txt";
                    }
                    FILE * fp;
                    fp = fopen(filename.c_str(), "w");
                    for (int j = 0; j < logger_coord.size(); ++j) {
                        fprintf(fp, "%lf\n", logger_coord[j][foot][i]);
                    }
                    fclose(fp);
                }
            }
        }

        void write_l_to_file() {
            for(int i = 0; i < 2; ++i) {
                if (logger_l[i].size() > 0) {
                    std::string filename = (std::string)("cavity_") + std::to_string(i) + "_l.txt";
                    FILE * fp;
                    fp = fopen(filename.c_str(), "w");
                    for (int j = 0; j < logger_l[i].size(); ++j) {
                        for (int k = 0; k < logger_l[i][j].size(); ++k) {
                            fprintf(fp, "%lf,", logger_l[i][j][k]);
                        }
                        fprintf(fp, "\n");
                    }
                }
            }
        }

        void write_angle_to_file() {
            for(int i = 0; i < 2; ++i) {
                if (logger_angle[i].size() > 0) {
                    std::string filename = (std::string)("cavity_") + std::to_string(i) + "_angle.txt";
                    FILE * fp;
                    fp = fopen(filename.c_str(), "w");
                    for (int j = 0; j < logger_angle[i].size(); ++j) {
                        for (int k = 0; k < logger_angle[i][j].size(); ++k) {
                            fprintf(fp, "%lf,", logger_angle[i][j][k]);
                        }
                        fprintf(fp, "\n");
                    }
                    fclose(fp);
                }
            }
        }

        void write_path_to_file() {
            std::string filename = (std::string)("path.txt");
            FILE * fp;
            fp = fopen(filename.c_str(), "w");
            for (int i = 0; i < logger_path.size(); ++i) {
                for (int j = 0; j < logger_path[i].size(); ++j) {
                    fprintf(fp, "%lf,", logger_path[i][j]);    
                }
                fprintf(fp, "\n");
            }
            fclose(fp);
        }
};

#endif
