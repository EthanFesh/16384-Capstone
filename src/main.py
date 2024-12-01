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
fa.open_gripper()
fa.reset_joints()


# Wrapper function that generates and follows trajectories to a desired pose
def go(end_pose):
    current_pose = np.eye(4)
    current_pose[:3, :3] = fa.get_pose().rotation
    current_pose[:3, 3] = fa.get_pose().translation
    cartesian_trajectory = TG.generate_straight_line(current_pose,end_pose)
    # print("++++++++++")
    # print(cartesian_trajectory)
    # print("++++++++++")
    joint_trajectory = TG.convert_cartesian_to_joint(cartesian_trajectory)
    # print("++++++++++")
    # print(joint_trajectory)
    # print("++++++++++")
    joint_trajectory = np.array(joint_trajectory)
    # TF.follow_joint_trajectory(TG.interpolate_joint_trajectory(joint_trajectory))

# read the pen holder position from pen_holder_pose.npy
pen_rot = fa.get_pose().rotation
pink_pen_xyz = np.load("pen_holder_pose.npy", allow_pickle=True)
'''TODO: update the displacement with the actual values'''
indigo_pen_xyz = pink_pen_xyz + np.array([0, 0, 0.1])
green_pen_xyz = pink_pen_xyz + np.array([0, 0, 0.2])

# Define pen poses (4x4 matrices)
pink_pen_pose = np.eye(4)
pink_pen_pose[:3, :3] = pen_rot
pink_pen_pose[:3, 3] = pink_pen_xyz

indigo_pen_pose = np.eye(4)
indigo_pen_pose[:3, :3] = pen_rot
indigo_pen_pose[:3, 3] = indigo_pen_xyz

green_pen_pose = np.eye(4)
green_pen_pose[:3, :3] = pen_rot
green_pen_pose[:3, 3] = green_pen_xyz

# Define drawing poses
whiteboard_pose = np.load("whiteboard_pose.npy", allow_pickle=True)

'''TODO: update the displacement with the actual value'''
line_1_start_pose = whiteboard_pose
line_1_end_pose = np.eye(4)
line_1_end_pose[:3, :3] = whiteboard_pose[:3, :3]
line_1_end_pose[:3, 3] = line_1_start_pose[:3, 3] + np.array([0.1, 0, 0])

# circle_start_pose = line_1_end_pose
# circle_xyzs = []
# '''TODO: generate points in the circle'''
# circle_poses = []
# for circle_xyz in circle_xyzs:
#     circle_pose = np.eye(4)
#     circle_pose[:3, :3] = whiteboard_pose[:3, :3]
#     circle_pose[:3, 3] = circle_xyz
#     circle_poses.append(circle_pose)

# '''TODO: update the displacement with the actual value'''
# print(len(circle_poses))
# line_2_start_pose = circle_poses[(len(circle_poses)-1)]
# line_2_end_pose = np.eye(4)
# line_2_end_pose[:3, :3] = whiteboard_pose[:3, :3]
# line_2_end_pose[:3, 3] = line_2_start_pose[:3, 3] + np.array([0.1, 0, 0])

# Define drop bin pose (4x4 matrix)
drop_xyz = np.load("drop_bin_pose.npy", allow_pickle=True)
drop_pose = np.eye(4)
drop_pose[:3, 3] = drop_xyz
'''TODO: not sure if the rotation should actually be the same'''
drop_pose[:3, :3] = pen_rot

print('Ready')
while (True):
    response = input("Press 'p' to pick up a pen, 'w' to write, 'd' to move pen over drop bin, 'q' to quit: ")
    if response == 'p':
        response = input("Press 'p' for pink pen, 'i' for indigo pen, 'g' for green pen: ")
        if (response == 'p'):
            go(pink_pen_pose)
        elif (response == 'i'):
            go(indigo_pen_pose)
        elif (response == 'g'):
            go(green_pen_pose)
        else:
            print('Invalid input')
        response = input("Press 'l' to lower arm: ")
        if (response == 'l'):
            current_xyz = fa.get_pose().translation
            '''TODO: update the displacement with the actual value'''
            current_xyz[2] -= 0.1
            lowered_pose = np.eye(4)
            lowered_pose[:3, :3] = pen_rot
            lowered_pose[:3, 3] = current_xyz
            go(lowered_pose)
        else:
            continue
        response = input("Press 'g' to grab pen: ")
        if (response == 'g'):
            fa.close_gripper()
        else:
            continue
        response = input("Press 'r' to raise arm: ")
        if (response == 'r'):
            current_xyz = fa.get_pose().translation
            '''TODO: update the displacement with the actual value'''
            current_xyz[2] += 0.1
            raised_pose = np.eye(4)
            raised_pose[:3, :3] = pen_rot
            raised_pose[:3, 3] = current_xyz
            go(raised_pose)
        else:
            continue
    elif response == 'w':
        response = input("Press '1' for first line, '2' for circle, '3' for second line: ")
        if response == '1':
            go(line_1_start_pose)
            response = input("Press '1' to start drawing: ")
            if (response == '1'):
                go(line_1_end_pose)
            else:
                continue
        elif response == '2':
            go(circle_start_pose)
            response = input("Press '1' to start drawing: ")
            if (response == '1'):
                cartesian_trajectory = TG.generate_curve(circle_poses)
                joint_trajectory = TG.convert_cartesian_to_joint(cartesian_trajectory)
                joint_trajectory = np.array(joint_trajectory)
                TF.follow_joint_trajectory(TG.interpolate_joint_trajectory(joint_trajectory))
            else:
                continue
        elif response == '3':
            go(line_2_start_pose)
            response = input("Press '1' to start drawing: ")
            if (response == '1'):
                go(line_2_end_pose)
            else:
                continue
        else:
            print('Invalid input')
    elif response == 'd':
        go(drop_pose)
        response = input("Press 'r' to release pen: ")
        if (response == 'r'):
            fa.open_gripper()
        else:
            continue
    elif response == 'q':
        break
    else:
        print('Invalid input')
    fa.reset_joints()
fa.reset_joints()
