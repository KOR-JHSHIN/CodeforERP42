#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix
from erp_driver.msg import erpStatusMsg, erpCmdMsg
import utm
from math import *


class PurePursuit:
    def __init__(self, waypoints):
        rospy.init_node("PurePursuit_node")
        self.ctrl_cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=10)
        self.gps_sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.gps_callback)
        self.utm_waypoints = utm_waypoints
        self.current_location = None
        self.look_ahead_distance = 5  #미정
        self.wheel_base = 1.040 # ERP42 축거


    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude] #rtk 위경도 수신
        self.current_utm_location = utm.from_latlon(self.current_location[0],self.current_location[1]) #수신한 위경도 utm 변환
        self.calculate_purepursuit()
        
        
    def find_nearest_waypoints(self): #현재위치에서 가장 가까운 waypoints 찾기
        nearest_distance = float('inf')
        nearest_waypoint = None


        for waypoint in self.utm_waypoints:
            distance = sqrt((self.current_utm_location[0] - utm_waypoints[0]) ** 2 +
                            (self.current_utm_location[1] - utm_waypoints[1]) ** 2) 
            
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_waypoint = waypoint
        

        return nearest_waypoint, nearest_distance
    
    def find_look_ahead_waypoint(self): # purepursuit의 목표지점으로 사용할 waypoint 찾기



    def calculate_purepursuit(self): #steer_angle 값 계산 delta= arctan((2*L*sin(alpha))/L_d)
        if self.current_location is None:
            return
        
        dx = target_point[0] - self.current_utm_location[0]
        dy = traget_point[0] - self.current_utm_location[1]
        alpha = atan2(dy/dx) - yaw # yaw값을 어떻게?? IMU도 사용해야할듯 
        pp_steer_angle = atan2(2*self.wheel_base*sin(alpha),self.look_ahead_distance)
        return pp_steer_angle



    def publish_ctrl_cmd(self)
        cmd_msg = erpCmdMsg()
        cmd_msg.speed = 0  
        cmd_msg.steer = pp_steer_angle
        cmd_msg.gear = 1  #
        cmd_msg.brake = 0
        cmd_msg.e_stop = False

        self.publisher.publish(cmd_msg)
       

def load_waypoints(filename):  # waypoints loading 함수
    waypoints = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            waypoints.append((float(row[0]), float(row[1])))
    return waypoints


def llh_to_utm(waypoints):  # loaded waypoints를 utm으로  
    utm_waypoints = []
    for lat, lon in waypoints:
        utm_coords = utm.from_latlon(lat, lon)
        utm_waypoints.append((utm_coords[0], utm_coords[1]))  # (Easting, Northing)만 저장
    return utm_waypoints


if __name__ == "__main__":
    try:
        waypoints = load_waypoints("gps_coordinates.csv")
        utm_waypoints = llh_to_utm(waypoints)
        pure_pursuit = PurePursuit(utm_waypoints)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
