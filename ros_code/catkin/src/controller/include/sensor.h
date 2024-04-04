#ifndef SENSOR
#define SENSOR

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
#include <math.h>
#include <atomic>
#include <stdio.h>

#include "common.h"

/**
 * @brief Sensor data (angle).
*/
typedef struct OptrackSensorData {
    DTYPE fore_coord[2];
    DTYPE hind_coord[2];
} OptrackSensorData;

class sensor {
    private:
        void subscribe_optrack();
        void optrack_callback(const std_msgs::String::ConstPtr& msg);

        std::thread *sub_optrack_t;
        DTYPE cur_fore_coord[2];
        DTYPE cur_hind_coord[2];
        std::atomic<bool> _lock;
    public:
        sensor();
        ~sensor();
        void get_fore_coord(DTYPE &global_z, DTYPE &global_x);
        void get_hind_coord(DTYPE &global_z, DTYPE &global_x);
        void get_global_theta(DTYPE &new_theta);
        void print_optrack();
};

/**
 * @brief String: splitting
 * 
 * @param str : Input string
 * @param pattern: Delimiter
 * @return: he result after string segmentation.
*/
inline std::vector<std::string> split(std::string str, std::string pattern) {
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;
  int size=str.size();
  for (int i = 0; i < size; ++i) {
    pos = str.find(pattern, i);
    if (pos < size) {
      std::string s = str.substr(i,pos-i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}

/**
 * @brief Decode the string ('data1, data2, data3, data4') into the corresponding coordinates in the opt_data structure.
 * 
 * @param data Input string.
 * @return OptrackSensorData
*/
inline OptrackSensorData decode_optrack(const std::string &data) {
    OptrackSensorData opt_data;
    auto res = split(data, ",");
    opt_data.fore_coord[0] = std::stod(res[0].c_str());
    opt_data.fore_coord[1] = std::stod(res[1].c_str());
    opt_data.hind_coord[0] = std::stod(res[2].c_str());
    opt_data.hind_coord[1] = std::stod(res[3].c_str());
    return opt_data;
}

inline sensor::sensor() {
    printf("[LOG INFO] Initialize the sensor reader\n");
    _lock = false;
    cur_fore_coord[0] = INF;
    cur_fore_coord[1] = INF;
    cur_hind_coord[0] = INF;
    cur_hind_coord[1] = INF;

    // Start the thread that subscribes to optrack data.
    sub_optrack_t = new std::thread(&sensor::subscribe_optrack, this);

    // Block until data is read.
    while(true) {
        if (cur_fore_coord[0] != INF) break;
    }
}

inline sensor::~sensor() {
    sub_optrack_t->join();
    delete sub_optrack_t;
}

inline void sensor::optrack_callback(const std_msgs::String::ConstPtr& msg) {
    auto optrack_coord = decode_optrack(msg->data);
    while (true) {
        if (_lock) continue;
        _lock = true;
        cur_fore_coord[0] = optrack_coord.fore_coord[0];
        cur_fore_coord[1] = optrack_coord.fore_coord[1];
        cur_hind_coord[0] = optrack_coord.hind_coord[0];
        cur_hind_coord[1] = optrack_coord.hind_coord[1];
        _lock = false;
        break;
    }    
}

inline void sensor::subscribe_optrack() {
    int argc = 0;
    ros::init(argc, nullptr, "controller_sub_optrack");
    ros::NodeHandle nh;
    ros::Subscriber ros_tutorial_sub = nh.subscribe("optrack_sensor", 1, &sensor::optrack_callback, this);
    ros::spin();
}

inline void sensor::get_fore_coord(DTYPE &global_z, DTYPE &global_x) {
    while (true) {
        if (_lock) continue;
        _lock = true;
        global_z = cur_fore_coord[0];
        global_x = cur_fore_coord[1];
        _lock = false;
        break;
    }
};

inline void sensor::get_hind_coord(DTYPE &global_z, DTYPE &global_x) {
    while (true) {
        if (_lock) continue;
        _lock = true;
        global_z = cur_hind_coord[0];
        global_x = cur_hind_coord[1];
        _lock = false;
        break;
    }
};

inline void sensor::get_global_theta(DTYPE &new_theta) {
    while (true) {
        if (_lock) continue;
        _lock = true;
        auto dz = cur_fore_coord[0] - cur_hind_coord[0];
        auto dx = cur_fore_coord[1] - cur_hind_coord[1];
        _lock = false;
        if (std::abs(dz) < LST) {
            new_theta = dx > 0 ? PI / 2 : PI * 3 / 2;
        } else if (std::abs(dx) < LST) {
            new_theta = dz > 0 ? 0 : PI;
        } else {
            new_theta = std::atan(abs(dx / dz));
            if (dz > 0 && dx > 0) {
                new_theta = new_theta;
            } else if (dz < 0 && dx > 0) {
                new_theta = PI - new_theta;
            } else if (dz < 0 && dx < 0) {
                new_theta = PI + new_theta;
            } else if (dz < 0 && dx > 0) {
                new_theta = 2 * PI - new_theta;
            }
        }
        break;
    }
}

inline void sensor::print_optrack() {
    printf("optrack forefoot:[%.4lf, %.4lf], hindfoot:[%.4lf, %.4lf]\n",    \
            cur_fore_coord[0], cur_fore_coord[1], cur_hind_coord[0], cur_hind_coord[1]);
}

#endif
