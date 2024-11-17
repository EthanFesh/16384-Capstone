"""
    You will use this script for the final demo.
    Implement your code flow here.
    Your main fucntion can take in different command line arguments to run different parts of your code.
"""
import argparse
import sys
from frankapy import FrankaArm
from frankapy import FrankaConstants as FC
import numpy as np
from motion_planner import TrajectoryFollower, TrajectoryGenerator

# Define default values
parser = argparse.ArgumentParser()
parser.add_argument('--foo', default=1, type=float, help='foo')

# Get the args container with default values
if __name__ == '__main__':
    args = parser.parse_args()  # get arguments from command line
else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments

# Define trajectory generator and follower objects
TG = TrajectoryGenerator()
TF = TrajectoryFollower()
print('Starting robot')
fa = FrankaArm()
fa.reset_joints()
fa.open_gripper()

# Wrapper function that generates and follows trajectories to a desired pose
def go(end_pose):
    TF.follow_joint_trajectory(TG.interpolate_joint_trajectory(TG.convert_cartesian_to_joint(TG.generate_straight_line(fa.get_pose(),end_pose))))

# read the pen holder pose from pen_holder_pose.npy
pink_pen = np.load("pen_holder_pose.npy")
indigo_pen = pink_pen
green_pen = pink_pen

drop_pose = np.load("drop_bin_pose.npy")
whiteboard_pose = np.load("whiteboard_pose.npy")

print('Ready')
while (True):
    response = input("Press 'p' to pick up a pen, 'w' to write, 'd' to move pen over drop bin, 'q' to quit: ")
    if response == 'p':
        response = input("Press 'p' for pink pen, 'i' for indigo pen, 'g' for green pen: ")
        if (response == 'p'):
            go(pink_pen)
        elif (response == 'i'):
            go(indigo_pen)
        elif (response == 'g'):
            go(green_pen)
        else:
            print('Invalid input')
        response = input("Press 'g' to grab pen: ")
        if (response == 'g'):
            fa.close_gripper()
    elif response == 'w':
        response = input("Press 'l' for line, 'c' for circle: ")
        go(whiteboard_pose)
    elif response == 'd':
        go(drop_pose)
        response = input("Press 'r' to release pen: ")
        if (response == 'r'):
            fa.open_gripper()
    elif response == 'q':
        break
    else:
        print('Invalid input')
    fa.reset_joints()
fa.reset_joints()
