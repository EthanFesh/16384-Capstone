import sys
sys.path.append("../config")

import numpy as np
from robot_config import RobotConfig
from task_config import TaskConfig
from autolab_core import RigidTransform
from frankapy import FrankaArm
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from utils import _slerp, _rotation_to_quaternion, _quaternion_to_rotation
from robot import Robot
import rospy

d = False

# Can choose to add another transform here to goto the center of grip
dh_parameters = np.array([
    [0, 0, 0.333, None],         # Joint 1
    [0, -np.pi/2, 0, None],      # Joint 2
    [0, np.pi/2, 0.316, None],   # Joint 3
    [0.0825, np.pi/2, 0, None],  # Joint 4
    [-0.0825, -np.pi/2, 0.384, None], # Joint 5
    [0, np.pi/2, 0, None],       # Joint 6
    [0.088, np.pi/2, 0, None],   # Joint 7
    [0, 0, 0.107, 0]            # Flange
])

class TrajectoryGenerator:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.max_vel = RobotConfig.MAX_VELOCITY
        self.max_acc = RobotConfig.MAX_ACCELERATION
    
    def generate_straight_line(self, start_pose, end_pose):
        """
        This function creates a smooth straight-line trajectory in Cartesian space
         Parameters
        ----------
        You can define any parameters you need for this function.
            
        Return
        ------
        array_like
            Input to either interpolate_cartesian_trajectory() or convert_cartesian_to_joint()

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        - Need start pose (4x4 matrix) and end pose (4x4 matrix)
        - Use linear interpolation for position: p(t) = p0 + t*(p1-p0)
        - Use SLERP (spherical linear interpolation) for rotation
        - Number of points should give enough resolution for smooth motion
        - Each waypoint should be a 4x4 transformation matrix
        """
        d_0 = start_pose[:3, 3]
        d_1 = end_pose[:3, 3]
        p_0 = start_pose[:3, :3]
        p_1 = end_pose[:3, :3]
        p_0 = _rotation_to_quaternion(p_0)
        p_1 = _rotation_to_quaternion(p_1)
        num_points = int(np.linalg.norm(d_1 - d_0) / TaskConfig.PATH_RESOLUTION)
        print("Num points:")
        print(num_points)
        input("Press enter to continue")
        cartesian_trajectory = []
        for i in range(num_points):
            t = i / num_points
            lerp = d_0 + t * (d_1 - d_0)
            slerp = _slerp(p_0, p_1, t)
            R = _quaternion_to_rotation(slerp)
            pose = np.eye(4)
            pose[:3, :3] = R
            pose[:3, 3] = lerp
            pose.tolist()
            cartesian_trajectory.append(pose)
        cartesian_trajectory = np.array(cartesian_trajectory)
        return cartesian_trajectory
        
    def generate_curve(self, poses):
        """
        This function creates a smooth curved trajectory in Cartesian space.

        Parameters
        ----------
        You can define any parameters you need for this function.
            
        Return
        ------
        array_like
            Input to either interpolate_cartesian_trajectory() or convert_cartesian_to_joint()

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        - Need list of points defining the curve
        - Can break curve into segments and use linear interpolation for each
        - Each waypoint is a 4x4 transformation matrix
        - Keep orientation aligned with curve direction
        - PATH_RESOLUTION from TaskConfig helps determine point spacing
        - Line segments should be small enough for smooth motion

        """
        start_idx = 0
        end_idx = 1
        cartesian_trajectory = []
        '''TODO: currently i'm assuming the poses passed in have the right spacing for smooth motion, and
        we can ensure that in main when we generate the poses, but feel free to add stuff here as needed'''
        while end_idx < len(poses):
            traj = self.generate_straight_line(poses[start_idx], poses[end_idx]).tolist()
            cartesian_trajectory.extend(traj)
            start_idx += 1
            end_idx += 1
        return np.array(cartesian_trajectory)
    
    def interpolate_cartesian_trajectory(self, cartesian_trajectory):
        """
        Time-parameterize Cartesian trajectory with trapezoidal velocity profile.

        Parameters
        ----------
        cartesian_trajectory : array_like
            Array of poses representing path in Cartesian space
        
        Returns
        ------- 
        array_like
            Time-parameterized trajectory with 20ms spacing

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        Key Requirements:  
        - Timing: Waypoints must be spaced exactly 20ms apart for controller
        - Safety: Stay within MAX_VELOCITY and MAX_ACCELERATION limits
        - Smoothness: Use trapezoidal velocity profile for acceleration/deceleration

        Implementation:
        - Calculate duration based on path length and velocity limits
        - Generate trapezoidal velocity profile with acceleration limits 
        - Ensure 20ms spacing between waypoints
        - For rotations: Use SLERP to interpolate between orientations
        """
        raise NotImplementedError("Implement interpolate_cartesian_trajectory")

    def interpolate_joint_trajectory(self, waypoints):
        """
        Time-parameterize joint trajectory with trapezoidal velocity profile.

        Parameters
        ----------
        waypoints : array_like 
            Joint space array of configurations

        Returns
        -------
        array_like
            Time-parameterized trajectory with 20ms spacing

        Raises
        ------
        NotImplementedErr--------i = "+str(i)+"--------")
                    print("t = " + str(t))
                    print("Waypoint i:")
                    print(waypoints[start_idx])
                    print("Waypoint i+1:")
                    print(waypoints[end_idx])
                    print("new pos :")
                    print(new_pos)
                    print("Updated trajectory:")
                    print(tmp_trajectory)or
            This function needs to be implemented.

        Hints
        -----
        Key Requirements:
        - Timing: Waypoints must be spaced exactly 20ms apart for controller
        - Safety: Stay within MAX_VELOCITY and MAX_ACCELERATION limits 
        - Smoothness: Use trapezoidal velocity profile for acceleration/deceleration

        Implementation:
        - Use max velocity and acceleration from RobotConfig
        - Ensure smooth acceleration and deceleration
        - Keep 20ms between waypoints as required by controller

        """
        start_idx = 0
        end_idx = 1
        print("----------Recieved trajectory----------")
        print(len(waypoints))
        print(waypoints)
        return_trajectory = []
        
        while end_idx < len(waypoints):
            '''TODO: not sure if we need to do something different now that we have more points 
            and are interpolating between each pair, I feel like this doesn't really implement the
            trapezoidal velocity, it's the same as what we did for the capstone but just between each
            pair of points instead of just a start and end point. but also
            even if this is right the number of points might also need to be tweaked. maybe like a function
            that depending on the distance between the start and end point determines the number of points'''
            num_points = 5
            tmp_trajectory = []
            for i in range(num_points+1):
                t = i/num_points
                new_pos = waypoints[start_idx]*(1-t) + waypoints[end_idx]*t
                new_pos = new_pos.tolist()
                tmp_trajectory.append(new_pos)
                if d :
                    print("--------i = "+str(i)+"--------")
                    print("t = " + str(t))
                    print("Waypoint i:")
                    print(waypoints[start_idx])
                    print("Waypoint i+1:")
                    print(waypoints[end_idx])
                    print("new pos :")
                    print(new_pos)
                    print("Updated trajectory:")
                    print(tmp_trajectory)
            return_trajectory.extend(tmp_trajectory)
            if d:
                print("--------ret_traj---------")
                print(return_trajectory)
            start_idx += 1
            end_idx += 1
        return return_trajectory
    
    def convert_cartesian_to_joint(self, cartesian_trajectory):
        """
        Convert Cartesian trajectory to joint             if np.any(config < RobotConfig.JOINT_LIMITS_MIN) or np.any(config > RobotConfig.JOINT_LIMITS_MAX):
                raise ValueError('Joint limits violated')
        Parameters
        ----------
        cartesian_trajectory : array_like
            Array of poses in Cartesian space

        Returndh_parameterss
        -------
        array_like
            Joint space trajectory

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        Key Requirements:
        - Safety: All soraiselutions must respect joint limits
        - Smoothness: Solutions should minimize joint motion between waypoints

        Implementation:
        - Use Jacobian pseudo-inverse method  
        - Check joint limits after IK
        - Use previous joint solution as seed for next IK
        """
        print("in cartesian to joint")
        joint_trajectory = []
        robot = Robot()
        # print(len(cartesian_trajectory))
        i = 0
        for pose in cartesian_trajectory:
            config = robot._inverse_kinematics(pose, joint_trajectory[-1] if joint_trajectory else None, dh_parameters)
            joint_trajectory.append(config)
            i = i + 1
            # if (i % 10 == 0):
            #     print(i)
        output = []
        none_counter = 0
        for pose in joint_trajectory:
            if not (pose is None):
                output.append(pose.tolist())
            else :
                none_counter += 1
                print(none_counter)
        return output

class TrajectoryFollower:
    def __init__(self):
        self.dt = 0.02  # Required 20ms control loop
        self.fa = FrankaArm()
        
    def follow_joint_trajectory(self, joint_trajectory):
        """
        Follow a joint trajectory using dynamic control.
        
        From writeup: Must have 20ms between waypoints and maintain smooth motion
        
        Parameters
        ----------
        joint_trajectory : np.ndarray
            Array of shape (N, 7) containing joint angles for each timestep
        """
        rospy.loginfo('Initializing Sensor Publisher')
        pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        rate = rospy.Rate(1 / self.dt)

        rospy.loginfo('Publishing joints trajectory...')
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(joint_trajectory[0], duration=1000, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()
        for i in range(1, len(joint_trajectory)):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=joint_trajectory[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            
            rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
            pub.publish(ros_msg)
            rate.sleep()

        # Stop the skill
        # Alternatively can call fa.stop_skill()
        term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
            )
        pub.publish(ros_msg)

        rospy.loginfo('Done')
