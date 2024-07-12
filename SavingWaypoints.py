#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import os
import csv
from math import sqrt
from pynput import keyboard

class SavingWaypoints:
    def __init__(self):
        rospy.init_node("saving_waypoints")
        self.gps_sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.waypointsaving_callback)
        self.previous_location = None
        self.csv_file = None
        file = open("gps_coordinates.txt", 'w')
        self.csv_file = csv.writer(file)
        self.save_waypoint = False  

        listener = keyboard.Listener(on_press=self.on_key_event)
        listener.start()

    def on_key_event(self, key):
        try:
            if key.char == 's':
                self.save_waypoint = True
        except AttributeError:
            pass

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

        if self.save_waypoint:
            data = [x, y]
            self.csv_file.writerow(data)
            print(f"location saved: {data}")
            self.previous_location = [x, y]
            self.save_waypoint = False
        else:
            print(f"location not saved: {x, y}")

if __name__ == "__main__":
    try:
        saving_waypoints = SavingWaypoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
