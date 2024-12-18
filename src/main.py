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
from robot import Robot
from motion_planner import TrajectoryFollower, TrajectoryGenerator

# Can choose to add another transform here to goto the center of grip
dh_parameters = np.array([
    [0, 0, 0.333, None],         # Joint 1
    [0, -np.pi/2, 0, None],      # Joint 2
    [0, np.pi/2, 0.316, None],   # Joint 3
    [0.0825, np.pi/2, 0, None],  # Joint 4
    [-0.0825, -np.pi/2, 0.384, None], # Joint 5
    [0, np.pi/2, 0, None],       # Joint 6
    [0.088, np.pi/2, 0, None],   # Joint 7
    [0, 0, 0.107, -np.pi/4],            # Flange
    [0, 0, 0.1034, 0] # Center of grip
])

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
robot = Robot()

def curve(circle_poses):
    print("trying to draw curve")
    joint_trajectory = TG.convert_cartesian_to_joint(circle_poses, False, fa.get_joints(), dh_parameters)
    joint_trajectory = np.array(joint_trajectory)
    interp_trajectory = TG.interpolate_joint_trajectory(joint_trajectory)
    TF.follow_joint_trajectory(interp_trajectory)
    # unconverged = Truepink
    # seed = fa.get_joints()
    # attempts = 0
    # while(unconverged and attempts < 5):
    #     try:
    #         joint_trajectory = TG.convert_cartesian_to_joint(circle_poses, True, seed)
    #         joint_trajectory = np.array(joint_trajectory)
    #         interp_trajectory = TG.interpolate_joint_trajectory(joint_trajectory)
    #         TF.follow_joint_trajectory(interp_trajectory)
    #         unconverged = False
    #     except:
    #         print('Ik did not converge')
    #         random_adjustment = np.random.uniform(low=0, high=0.0001, size=(7,))
    #         seed = seed + random_adjustment
    #         attempts += 1

# Wrapper function that generates and follows trajectories to a desired pose
def go(end_pose, line=False):
    current_pose = np.eye(4)
    current_pose = robot.forward_kinematics_v2(dh_parameters, fa.get_joints())
    cartesian_trajectory = TG.generate_straight_line(current_pose, end_pose)
    unconverged = True
    seed = fa.get_joints()
    attempts = 0
    while(unconverged and attempts < 5):
        try:
            joint_trajectory = TG.convert_cartesian_to_joint(cartesian_trajectory, line, seed, dh_parameters)
            joint_trajectory = np.array(joint_trajectory)
            interp_trajectory = TG.interpolate_joint_trajectory(joint_trajectory)
            TF.follow_joint_trajectory(interp_trajectory)
            unconverged = False
        except:
            print('Ik threw error')
            random_adjustment = np.random.uniform(low=0, high=0.0001, size=(7,))
            seed = seed + random_adjustment
            attempts += 1

# read the pen holder position from pen_holder_pose.npy
pen_rot = robot.forward_kinematics_v2(dh_parameters, fa.get_joints())
# print("pen_rot", pen_rot)
pen_rot = pen_rot[:3, :3]
# priS_rot)
pink_pen_xyz = np.load("pink_pen_pose.npy", allow_pickle=True)
indigo_pen_xyz = np.load("indigo_pen_pose.npy", allow_pickle=True)
green_pen_xyz = np.load("green_pen_pose.npy", allow_pickle=True)
blue_pen_xyz = np.load("blue_pen_pose.npy", allow_pickle=True)

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

blue_pen_pose = np.eye(4)
blue_pen_pose[:3, :3] = pen_rot
blue_pen_pose[:3, 3] = blue_pen_xyz

# Define drawing poses
whiteboard_pose = np.load("whiteboard_pose.npy", allow_pickle=True)

# Define drop bin pose (4x4 matrix)
drop_pose = np.load("drop_bin_pose.npy", allow_pickle=True)

print('Ready')
while (True):
    response = input("Press 'p' to pick up a pen, 'w' to write, 'd' to move pen over drop bin, 'h' to put pen back in holder', 'q' to quit: ")
    if response == 'p':
        response = input("Press 'p' for pink pen, 'i' for indigo pen, 'g' for green pen, 'b' for blue pen: ")
        color = ""
        if (response == 'p'):
            color = "pink"
            go(pink_pen_pose)
        elif (response == 'i'):
            color = "indigo"
            go(indigo_pen_pose)
        elif (response == 'g'):
            color = "green"
            go(green_pen_pose)
        elif (response == 'b'):
            color = "blue"
            go(blue_pen_pose)
        else:
            print('Invalid input')
        adjust = True
        while (adjust):
            response = input("Press 'a' to adjust gripper position, 'c' to continue: ")
            if (response == 'a'):
                response = input("Enter x, y, z: ")
                xyz_strings = np.array(response.split(" "))
                xyz = [i.astype(float) for i in xyz_strings]
                if (color == "pink"):
                    xyz = xyz + pink_pen_xyz
                    pink_pen_xyz += xyz
                elif (color == "indigo"):
                    xyz = xyz + indigo_pen_xyz
                    indigo_pen_xyz += xyz
                elif (color == "green"):
                    xyz = xyz + green_pen_xyz
                    green_pen_xyz += xyz
                else:
                    xyz = xyz + blue_pen_xyz
                    blue_pen_xyz += xyz
                goal_pose = np.eye(4)
                goal_pose[:3, :3] = pen_rot
                goal_pose[:3, 3] = xyz
                go(goal_pose)
            elif (response == 'c'):
                adjust = False
            else:
                print('Invalid input')
        response = input("Press 'g' to grab pen: ")
        if (response == 'g'):
            fa.close_gripper()
        else:
            continue
        response = input("Press 'r' to raise arm: ")
        if (response == 'r'):
            current_xyz = fa.get_pose().translation
            current_xyz[2] += 0.3
            raised_pose = np.eye(4)
            raised_pose[:3, :3] = pen_rot
            raised_pose[:3, 3] = current_xyz
            go(raised_pose)
        else:
            continue
    elif response == 'w':
        go(whiteboard_pose)
        adjust = True
        goal_pose = whiteboard_pose
        while (adjust):
            response = input("Press 'a' to adjust pen position, 'c' to continue: ")
            if (response == 'a'):
                response = input("Enter x, y, z: ")
                xyz_strings = np.array(response.split(" "))
                xyz = [i.astype(float) for i in xyz_strings]
                xyz = np.append(xyz, 1.0)
                transformed = goal_pose @ xyz
                goal_pose = np.eye(4)
                goal_pose[:3, :3] = whiteboard_pose[:3, :3]
                goal_pose[:3, 3] = transformed[:3]
                go(goal_pose)
            elif (response == 'c'):
                adjust = False
            else:
                print('Invalid input')
        circle1_start_pose = goal_pose
        circle1_xyzs = []
        for i in range(64):
            point = (np.array([0.05*np.cos(i*2*np.pi/128) - 0.05, 0.05*np.sin(i*2*np.pi/128), 0, 1]))
            transformed = goal_pose @ point
            transformed = transformed[:3]
            circle1_xyzs.append(transformed)
        circle1_poses = []
        for circle_xyz in circle1_xyzs:
            circle_pose = np.eye(4)
            circle_pose[:3, :3] = whiteboard_pose[:3, :3]
            circle_pose[:3, 3] = circle_xyz
            circle1_poses.append(circle_pose)
        circle2_start_pose = goal_pose
        circle2_xyzs = []
        for i in range(64):
            point = (np.array([0.05*np.cos(i*2*np.pi/128) - 0.05, -0.05*np.sin(i*2*np.pi/128), 0, 1]))
            transformed = goal_pose @ point
            transformed = transformed[:3]
            circle2_xyzs.append(transformed)
        circle2_poses = []
        for circle_xyz in circle2_xyzs:
            circle_pose = np.eye(4)
            circle_pose[:3, :3] = whiteboard_pose[:3, :3]
            circle_pose[:3, 3] = circle_xyz
            circle2_poses.append(circle_pose)
        circle3_xyzs = []
        for i in range(64):
            point = (np.array([0.03*np.cos(i*2*np.pi/128) - 0.05, 0.06*np.sin(i*2*np.pi/128), 0, 1]))
            transformed = goal_pose @ point
            transformed = transformed[:3]
            circle3_xyzs.append(transformed)
        circle3_poses = []
        for circle_xyz in circle3_xyzs:
            circle_pose = np.eye(4)
            circle_pose[:3, :3] = whiteboard_pose[:3, :3]
            circle_pose[:3, 3] = circle_xyz
            circle3_poses.append(circle_pose)
        line_1_start_pose = goal_pose
        line1_poses = []
        for i in range(50):
            displacement = np.array([-0.003*i, 0, 0, 1])
            transformed = goal_pose @ displacement
            new_pose = np.eye(4)
            new_pose[:3, :3] = line_1_start_pose[:3, :3]
            new_pose[:3, 3] = transformed[:3]
            line1_poses.append(new_pose)
        line_2_start_pose = goal_pose
        line2_poses = []
        for i in range(50):
            displacement = np.array([0, 0.003*i, 0, 1])
            transformed = goal_pose @ displacement
            new_pose = np.eye(4)
            new_pose[:3, :3] = line_2_start_pose[:3, :3]
            new_pose[:3, 3] = transformed[:3]
            line2_poses.append(new_pose)
        line_3_start_pose = goal_pose
        line3_poses = []
        for i in range(32):
            displacement = np.array([-0.003 * i, 0.003*i, 0, 1])
            transformed = goal_pose @ displacement
            new_pose = np.eye(4)
            new_pose[:3, :3] = line_3_start_pose[:3, :3]
            new_pose[:3, 3] = transformed[:3]
            line3_poses.append(new_pose)
        line_4_start_pose = goal_pose
        line4_poses = []
        for i in range(32):
            displacement = np.array([-0.003 * i, -0.003*i, 0, 1])
            transformed = goal_pose @ displacement
            new_pose = np.eye(4)
            new_pose[:3, :3] = line_3_start_pose[:3, :3]
            new_pose[:3, 3] = transformed[:3]
            line4_poses.append(new_pose)
        response = input("Press '1' for first line, '2' for second line, '3' for third line , '4' for first curve, '5' for second curve, '6' for third curve: , '7' for last line: ")
        if response == '1':
            curve(line1_poses)
        elif response == '2':
            # cartesian_trajectory = TG.generate_curve(circle_poses)
            # cartesian_trajectory = TG.interpolate_cartesian_trajectory(cartesian_trajectory)
            curve(line2_poses)
            # wait for user input to continue
            # input("Press Enter to continue")
        elif response == '3':
            curve(line3_poses)  
            # curve(circle3_poses)
            # go(line_2_end_pose, True)
        elif response == '4':
            curve(circle1_poses)
            # go(line_3_end_pose, True)
        elif response == '5':
            curve(circle2_poses)
        elif response == '6':
            curve(circle3_poses)
        elif response == '7':
            curve(line4_poses)
        else:
            print('Invalid input')
    elif response == 'd':
        go(drop_pose)
        adjust = True
        while (adjust):
            response = input("Press 'a' to adjust drop position, 'c' to continue: ")
            if (response == 'a'):
                response = input("Enter x, y, z: ")
                xyz_strings = np.array(response.split(" "))
                xyz = [i.astype(float) for i in xyz_strings]
                xyz = np.append(xyz, 1.0)
                transformed = drop_pose @ xyz
                goal_pose = np.eye(4)
                goal_pose[:3, :3] = drop_pose[:3, :3]
                goal_pose[:3, 3] = transformed[:3]
                go(goal_pose)
            elif (response == 'c'):
                adjust = False
            else:
                print('Invalid input')
        response = input("Press 'r' to release pen: ")
        if (response == 'r'):
            fa.open_gripper()
        else:
            continue
    elif response == 'q':
        break
    elif response == 'h':
        place_pose = np.eye(4)
        place_pose[:3, :3] = pen_rot
        place_pose[:3, 3] = blue_pen_pose[:3, 3] + np.array([0, 0, 0.1])
        go(place_pose)
        adjust = True
        while (adjust):
            response = input("Press 'a' to adjust drop position, 'c' to continue: ")
            if (response == 'a'):
                response = input("Enter x, y, z: ")
                xyz_strings = np.array(response.split(" "))
                xyz = np.array([i.astype(float) for i in xyz_strings])
                goal_pose = np.eye(4)
                goal_pose[:3, :3] = pen_rot
                goal_pose[:3, 3] = place_pose[:3, 3] + xyz
                go(goal_pose)
            elif (response == 'c'):
                adjust = False
            else:
                print('Invalid input')
        response = input("Press 'r' to release pen: ")
        if (response == 'r'):
            fa.open_gripper()
        else:
            continue
    else:
        print('Invalid input')
    fa.reset_joints()
fa.reset_joints()