#!/usr/bin/env python
#coding=utf-8
import rospy
from std_msgs.msg import String
import socket
import json
import math

'''
    opt_data.fore_coord[0] = std::stod(res[0].c_str());
    opt_data.fore_coord[1] = std::stod(res[1].c_str());
    opt_data.hind_coord[0] = std::stod(res[2].c_str());
    opt_data.hind_coord[1] = std::stod(res[3].c_str());
'''
marker_ids = ['53905', '53989'] #(front_foot, rear_foot)


first_point = True
root_coord = []
theta = 0

def read_sensor_from_optrack(s):
    data, addr = s.recvfrom(1024)
    json_data = data.decode().split('\n')
    dict_data = json.loads(json_data[-1])
    data = dict_data['markers']
    coord_str = ""
    coord = list()
    for marker_id in marker_ids:
        coord.append(data[marker_id])
    
    global first_point
    global root_coord
    global theta

    if first_point == True:
        root_coord = coord[-1]
        fore_coord = coord[0]
        dx = fore_coord[0] - root_coord[0]
        dz = fore_coord[2] - root_coord[2]
        theta = math.atan(math.fabs(dx) / math.fabs(dz))
        if dz < 0 and dx > 0:
            theta = math.pi - theta
        if dz < 0 and dx < 0:
            theta = math.pi + theta
        if dz > 0 and dx < 0:
            theta = -theta
        first_point = False
        root_coord = coord[0]
        #theta += math.pi
    else:
        for i in range(len(coord)):
            dz = coord[i][2] - root_coord[2]
            dx = coord[i][0] - root_coord[0]
            coord[i][2] = dz * math.cos(-theta) - dx * math.sin(-theta)
            coord[i][0] = dz * math.sin(-theta) + dx * math.cos(-theta)
            coord_str += str(coord[i][2]) + ',' + str(coord[i][0]) + ','

    return coord_str

def sensor_reader():
    pub = rospy.Publisher('optrack_sensor', String, queue_size = 5)
    rospy.init_node('optrack_sensor_reader', anonymous = True)
    rate = rospy.Rate(50) # 50hz
    
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('10.0.66.100', 12345))
    #s.bind(('127.0.0.1', 12345))
    count = 0
    while not rospy.is_shutdown():
        data = read_sensor_from_optrack(s)
        if count % 20 == 0:
            rospy.loginfo(data)
        count += 1
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_reader()
    except rospy.ROSInterruptException:
        pass
