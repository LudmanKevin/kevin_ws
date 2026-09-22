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

def main(args=None):
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

        p1= posj(15.91, 42.06, -118.97, -2.94, -102.61, 104.61)
        p2= posj(0, 0, 0, 0, 0, 0)

        movej(p2, vel=100, acc=50)
        movej(p1, vel=100, acc=50)

        while rclpy.ok():
            movej(p2, vel=100, acc=50)
            movej(p1, vel=100, acc=50)
            break

        rclpy.shutdown()

if __name__ == "__main__":
        main()