import serial
# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64MultiArray
import pymap3d as pm
import numpy as np


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        
        port = '/dev/ttyGPS'
        # baudrate = 38400
        baudrate = 460800
        
        self.dt = 0.1
        
        self.gps_lon_pub = self.create_publisher(String, '/gps/lon', 1)
        self.gps_lat_pub = self.create_publisher(String, '/gps/lat', 1)
        self.gps_spd_pub = self.create_publisher(String, '/gps/spd', 1)
        self.gps_hdg_pub = self.create_publisher(String, '/gps/hdg', 1)
        
        self.gps_timer = self.create_timer(0.1, self.pub_gps)

        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Serial connection established on port {port}")
        except serial.SerialException:
            self.get_logger().error(f"Failed to open serial port {port}")
            return

        # Start reading data
        self.read_data()

    def read_data(self):
        while rclpy.ok():
            try:
                line = self.serial.readline()
                line = line.decode().strip()  # Convert bytes to string
                # print(line)
                if line.startswith("$GNGLL"):  # Check string representation
                    data = line.split(',')
                    latitude = float(data[1])  # Latitude
                    longitude = float(data[3])  # Longitude
                    # self.get_logger().info(f"Latitude: {latitude}, Longitude: {longitude}")
                    
                    lat_degrees = int(latitude / 100)
                    lat_minutes = latitude % 100
                    lat_decimal_degrees = lat_degrees + (lat_minutes / 60)

                    lon_degrees = int(longitude / 100)
                    lon_minutes = longitude % 100
                    lon_decimal_degrees = lon_degrees + (lon_minutes / 60)

                    print("Latitude: ", lat_decimal_degrees, "Longitude: ", lon_decimal_degrees)
                    self.lat_decimal_degrees = lat_decimal_degrees
                    self.lon_decimal_degrees = lon_decimal_degrees
                if line.startswith("$GNVTG") or line.startswith("$GPVTG"):
                    data = line.split(',')
                    if len(data[3]) != 0: # NED -> ENU coordinate
                        hdg = float(data[6])
                        if hdg > 180:
                            hdg -= 360
                        hdg = -hdg
                        self.heading = hdg
                        print("heading[deg]:", hdg)
                    if len(data[5]) != 0:
                        spd = float(data[5])
                        spd = spd * 0.5144
                        self.speed = spd
                        print("speed: ", spd)
                    
            except serial.SerialException:
                self.get_logger().error(f"Serial connection error")
                break

        if not rclpy.ok():
            print("not ok")
            rclpy.spin_once(self)
            
    def pub_gps(self):
        if self.lat_decimal_degrees and self.lon_decimal_degrees:
            lat_pub = String()
            lat_pub.data = self.lat_decimal_degrees
            lon_pub = String()
            lon_pub.data = self.lon_decimal_degrees
            self.gps_lat_pub.publish(lat_pub)
            self.gps_lon_pub.publish(lon_pub)
        else:
            return
        if self.heading:
            gps_hdg_pub = String()
            gps_hdg_pub.data = self.heading
            self.gps_hdg_pub.publish(gps_hdg_pub)
        if self.speed:
            gps_spd_pub = String()
            gps_spd_pub.data = self.speed
            self.gps_spd_pub.publish(gps_spd_pub)
            
def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
