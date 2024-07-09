#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import os
import csv
from math import sqrt

class WaypointsSaving:
    def __init__(self):
        rospy.init_node("saving_waypoints")
        self.gps_sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.waypointsaving_callback)
        self.previous_location = None
        self.csv_file = None
        file = open("gps_coordinates.txt", 'w')
        self.csv_file = csv.writer(file)

    def waypointsaving_callback(self, msg):
        gps_msg = msg

        os.system("clear")
        print("------------------------------")
        print(f"latitude : {gps_msg.latitude}")
        print(f"longitude : {gps_msg.longitude}")
        print(f"altitude : {gps_msg.altitude}")
        print("------------------------------")

        coord = (gps_msg.latitude, gps_msg.longitude)
        x = coord[0]
        y = coord[1]

        if self.previous_location is not None:
         lf.previous_location[1])**2) * 111000
            distance = sqrt((x - self.previous_location[0])**2 + (y - self.previous_location[1])**2)

            if distance >= 0.000045: # 5미터 정도 이동시 waypoints 저장 
                data = [x, y]
                self.csv_file.writerow(data)
                print(f"location : {data}")
                self.previous_location = [x, y]
            else:
                print(f"location not moved enough to record: {x, y}")
        else:
            data = [x, y]
            self.csv_file.writerow(data)
            print(f"initial location : {data}")
            self.previous_location = [x, y]

if __name__ == "__main__":
    try:
        waypoints_saving = WaypointsSaving()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
