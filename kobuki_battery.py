#!usr/bin/env python

'''
2025, Daniel Soriano Ponce
Affiliation: Case Western Reserve University - SaPHaRI Lab
Project: TurtleDock

This script continuously monitors the battery percentage and charging status of the Kobuki TurtleBot base and logs the
data into a timestamped CSV file on the host system. The data will be used later for performance monitoring, triggering behavior, or system diagnotics.

Built using ROS Noetic and tested on Raspberry Pi running Ubuntu 20.04.
'''

import roslib
import rospy
import csv
import os
import threading #added 07.05
import time #added 07.05
from datetime import datetime
from kobuki_msgs.msg import SensorState

class kobuki_battery():
    #maximum battery charge value for kobuki
    max_charge = 160

    def __init__(self):
        #initialize this script as a ROS node
        rospy.init_node("kobuki_battery")

        #dynamically determine home directory
        home = os.path.expanduser('~')
        self.csv_file = open(os.path.join(home, 'battery_log.csv'), 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        #write headers in case the file is empty
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow(['Timestamp', 'Battery (%)', 'Charging Status'])

        #subscribing to mobile base's sensor information
        rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.SensorPowerEventCallback)

        rospy.spin()

    def messenger(self, data):
        battery_percent = round(float(data.battery) / float(self.max_charge) * 100)

        #'data.charger' is non-zero when the turtlebot is charging or docked
        charging_status = "Charging" if int(data.charger) != 0 else "Not Charging"

        rospy.loginfo(f"Kobuki's battery is now: {battery_percent}%")
        rospy.loginfo(charging_status + " at docking station")

        # Log to CSV
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.csv_writer.writerow([timestamp, battery_percent, charging_status])
        self.csv_file.flush()  # Ensures data is written immediately

    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()

if __name__ == '__main__':
    try:
        kobuki_battery()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")



