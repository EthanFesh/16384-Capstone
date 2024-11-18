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
        d_0 = start_pose.translation
        d_1 = end_pose.translation
        p_0 = start_pose.rotation
        p_1 = end_pose.rotation
        p_0 = _rotation_to_quaternion(p_0)
        p_1 = _rotation_to_quaternion(p_1)
        # print(d_0, d_1, start_pose[:3, :3], end_pose[:3, :3])
        num_points = int(np.linalg.norm(d_1 - d_0) / TaskConfig.PATH_RESOLUTION)
        # print(num_points)
        cartesian_trajectory = []
        for i in range(num_points):
            t = i / num_points
            lerp = d_0+ t * (d_1 - d_0)
            slerp = _slerp(p_0, p_1, t)
            R = _quaternion_to_rotation(slerp)
            pose = RigidTransform(rotation=R, translation=lerp)
            cartesian_trajectory.append(pose)
            # cartesian_trajectory.append(np.block([[R, lerp.reshape(3, 1)], [0, 0, 0, 1]]))
        # print(cartesian_trajectory[-1])
        return cartesian_trajectory
        
    def generate_curve(self):
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
        raise NotImplementedError("Implement generate_curve")
    
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
    
    def get_distances(self, waypoints):
        distances = [0]
        for i in (len(waypoints)):
            distance = np.linalg.norm(waypoints[i+1] - waypoints[i]) + distances[-1]
            distances.append(distance)
        return distances
    
    def get_v_cruise(self, d):
        accel_d = self.max_vel ** 2 / (2 * self.max_acc)
        while (accel_d >= d/2):
            v_max = v_max/2
            accel_d = v_max ** 2 / (2 * self.max_acc)
        return v_max
    
    # def get_times(self, v_cruise, distances, waypoints):
    #     total_distance = distances[-1]
    #     accel_distance = v_cruise ** 2 / (2 * self.max_acc)
    #     cruise_distance = total_distance - accel_distance
    #     times = [0]
    #     v_i = 0
    #     for i in range(1, len(waypoints)):
    #         distance = distances[i]
    #         if distance <= accel_distance:
    #             # accelerating
    #             t = (-v_i + np.sqrt(v_i ** 2 + 2 * self.max_acc * distance)) / self.max_acc
    #             times.append(t)
    #             v_i = self.max_acc * t
    #         elif distance <= cruise_distance:
    #             # cruising
    #             t = (distance - accel_distance) / v_cruise
    #             times.append(t + times[-1])
    #         else:
    #             # decelerating
    #             t = (-v_i + np.sqrt(v_i ** 2 - 2 * self.max_acc * (total_distance - distance))) / - self.max_acc
    #             times.append(t + times[-1])
    #             v_i = v_i - self.max_acc * t
    #     return times  

    def interpolate_joint_trajectory(self, joint_trajectory):
        """
        Time-parameterize joint trajectory with trapezoidal velocity profile.

        Parameters
        ----------
        joint_trajectory : array_like 
            Array of joint angles

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
        - Use max velocity and acceleration from RobotConfig
        - Ensure smooth acceleration and deceleration
        - Keep 20ms between waypoints as required by controller

        """
        print("interpolate_joint_trajectory")
        frequency = self.dt
        duty_cycle = 0.25
        num_waypoints = len(joint_trajectory)
        num_joints = 7
        waypoints = np.array(joint_trajectory)
        times = np.linspace(0, num_waypoints * frequency, num_waypoints)
        num_points_per_segment = []
        num_segments = num_waypoints - 1
        for segment in range(num_segments):
            dt = times[0, segment + 1] - times[0, segment]
            num_points_per_segment.append(int(dt * frequency))

        # Pre-allocate trajectory matrix
        trajectory = np.zeros((7, int(np.sum(num_points_per_segment))))

        # Fill in trajectory segment-by-segment
        segment_start_point = 0
        for segment in range(num_segments):
            points_in_segment = num_points_per_segment[segment]
            segment_end_point = segment_start_point + points_in_segment

            num_ramp_points = int(duty_cycle * points_in_segment)
            ramp_time = (times[0, segment + 1] - times[0, segment]) * duty_cycle

            # --------------- BEGIN STUDENT SECTION ----------------------------------
            # TODO: Calculate the maximum velocity for this segment
            vm = np.zeros(num_joints)
            for joint in range(num_joints):
                q0 = waypoints[joint, segment]
                qf = waypoints[joint, segment + 1]
                displacement = qf - q0
                total_time = times[0, segment + 1] - times[0, segment]
                vm[joint] = displacement / (total_time - ramp_time)

            # TODO: Fill in the points for this segment of the trajectory
            # You need to implement the trapezoidal velocity profile here
            # Hint: Use three phases: ramp up, constant velocity, and ramp down

            # Example structure (you need to fill in the correct calculations):
            for joint in range(num_joints):
                q0 = waypoints[joint, segment]
                qf = waypoints[joint, segment + 1]
                v_max = vm[joint]
                a_max = v_max / ramp_time

                for i in range(points_in_segment):
                    t = i / frequency
                    if i < num_ramp_points:
                        # TODO: Implement ramp up phase
                        q = q0 + 0.5 * a_max * t**2
                    elif i >= points_in_segment - num_ramp_points:
                        # TODO: Implement ramp down phase
                        time_from_end = (points_in_segment - i) / frequency
                        q = qf - 0.5 * a_max * time_from_end**2
                    else:
                        # TODO: Implement constant velocity phase
                        q_ramp_end = q0 + 0.5 * v_max * ramp_time
                        q = q_ramp_end + v_max * (t - ramp_time)

                    trajectory[joint, segment_start_point + i] = q

            # --------------- END STUDENT SECTION ------------------------------------

            segment_start_point += points_in_segment

        return trajectory
        # num_segments = num_waypoints - 1
        # num_points_per_segment = []
        # for segment in range(num_segments):
        #     delta_t = times[segment + 1] - times[segment]
        #     num_points_per_segment.append(int(delta_t * self.dt))
        # waypoints = []
        # total_distance = distances[-1]
        # accel_distance = v_cruise ** 2 / (2 * self.max_acc)
        # cruise_distance = total_distance - accel_distance
        # for segment in range(num_segments):
        #     points_in_segment = num_points_per_segment[segment]
        #     distance = distances[segment]
        #     status = None
        #     if distance <= accel_distance:
        #         status = "accel"
        #     elif distance <= cruise_distance:
        #         status = "cruise"
        #     else:
        #         status = "decel"
        #     for i in range(points_in_segment):
        #         t = i * self.dt
        #         if status == "accel":
        #             waypoints.append(joint_trajectory[segment] + 0.5 * self.max_acc * t ** 2)
        #         elif status == "cruise":
        #             waypoints.append(joint_trajectory[segment] + v_cruise * t)
        #         else:
        #             t = points_in_segment - 1 - i * self.dt
        #             waypoints.append(joint_trajectory[segment] + 0.5 * self.max_acc * t ** 2)
    
    def convert_cartesian_to_joint(self, cartesian_trajectory):
        """
        Convert Cartesian trajectory to joint             if np.any(config < RobotConfig.JOINT_LIMITS_MIN) or np.any(config > RobotConfig.JOINT_LIMITS_MAX):
                raise ValueError('Joint limits violated')
        Parameters
        ----------
        cartesian_trajectory : array_like
            Array of poses in Cartesian space

        Returns
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
        - Safety: All solutions must respect joint limits
        - Smoothness: Solutions should minimize joint motion between waypoints

        Implementation:
        - Use Jacobian pseudo-inverse method  
        - Check joint limits after IK
        - Use previous joint solution as seed for next IK
        """
        print("in cartesian to joint")
        joint_trajectory = []
        robot = Robot()
        print(len(cartesian_trajectory))
        i = 0
        for pose in cartesian_trajectory:
            config = robot._inverse_kinematics(pose, joint_trajectory[-1] if joint_trajectory else None)
            joint_trajectory.append(config)
            i = i + 1
            if (i % 10 == 0):
                print(i)
        return joint_trajectory

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
