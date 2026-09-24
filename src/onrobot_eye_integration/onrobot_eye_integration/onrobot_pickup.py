import rclpy
import os
import sys

from rclpy.logging import get_logger

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

logger = get_logger('single_robot_simple_logger')

from .onrobot import RG
import argparse

def get_options():
        parser = argparse.ArgumentParser(description='Set options.')
        parser.add_argument('--gripper', dest='gripper', type=str, default="rg2", choices=['rg2', 'rg6'], help='Gripper type')
        parser.add_argument('--ip', dest='ip', type=str, default="192.168.1.1", help='IP address')
        parser.add_argument('--port', dest='port', type=str, default="502", help='Port number')
        return parser.parse_args()
        
def main(args=None):
        options = get_options()
        rg = RG(options.gripper, options.ip, options.port)
        rclpy.init(args=args)

        node = rclpy.create_node('single_robot_simple_py', namespace=ROBOT_ID)

        DR_init.__dsr__node = node
        
        try:
                from DSR_ROBOT2 import print_ext_result, movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb, set_velx, set_accx, set_robot_mode
                from DSR_ROBOT2 import posj, posx, posb
                from DSR_ROBOT2 import DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return
        
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        set_velx(30, 20) # (mm/sec) (deg/sec)
        set_accx(60, 40) # (mm/sec^2) (deg/sec^2)

        p1= posj(-4.89, 1.78, 101.17, 3.36, 75.28, -4.83)
        p2= posj(0, 0, 0, 0, 0, 0)

        rg.open_gripper()
        rg.close_gripper()

        while rclpy.ok():
                movej(p1, vel=100, acc=50)
                rg.open_gripper()
                #if 

        rclpy.shutdown()

if __name__ == "__main__":
        main()