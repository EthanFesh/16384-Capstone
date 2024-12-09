import sys
sys.path.append('../config')
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from robot_config import RobotConfig
from task_config import TaskConfig
from utils import _compute_rotation_error

d = False

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7
    
    def forward_kinematics_v1(self, thetas):
        """
        Compute foward kinematics
        
        Your implementation should:
        1. Compute transformation matrices for each frame using DH parameters
        2. Compute end-effector pose
        
        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters (you can choose to apply the offset to the tool flange or the pen tip)
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            End-effector pose
        """
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO
        dh_parameters = self._get_dh_parameters(thetas)
        frames = np.zeros((4, 4, len(dh_parameters)+1))
        frames[:, :, 0] = np.eye(4)
        H = np.eye(4)

        for i in range(self.dof):
            a, alpha, d, theta = dh_parameters[i,:]

            Rot_x = np.array([
                [1,0,0,0],
                [0,np.cos(alpha),-np.sin(alpha),0],
                [0,np.sin(alpha),np.cos(alpha),0],
                [0,0,0,1]
            ])
            Trans_x = np.array([
                [1,0,0,a],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]
            ])
            Trans_z = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,d],
                [0,0,0,1]
            ])
            Rot_z = np.array([
                [np.cos(theta),-np.sin(theta),0,0],
                [np.sin(theta),np.cos(theta),0,0],
                [0,0,1,0],
                [0,0,0,1]
            ])

            H_i = Rot_x @ Trans_x @ Trans_z @ Rot_z

            H = H @ H_i

            frames[:,:,i+1] = H 

        return frames

        # --------------- END STUDENT SECTION --------------------------------------------------
    
    def jacobians_v1(self, thetas):
        """
        Compute the Jacobians for each frame.

        Parameters
        ----------
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            Jacobians
        """
        if thetas.shape != (self.dof,):
            raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')

        jacobians = np.zeros((6, self.dof, self.dof + 1))
        epsilon = 1e-6

        # --------------- BEGIN STUDENT SECTION ----------------------------------------
        # TODO: Implement the numerical computation of the Jacobians

        # Hints:
        # - Perturb each joint angle by epsilon and compute the forward kinematics.
        # - Compute numerical derivatives for x, y, and positions.
        # - Determine the rotational component based on joint contributions.

        # if thetas.shape != (self.dof,):
        #     raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')

        # jacobians = np.zeros((6, self.dof, self.dof + 1))
        # frames = self.forward_kinematics(thetas)
        # for j in range(self.dof + 1):
        #     o_j = frames[0:3, 3, j]
        #     for i in range(self.dof):
        #         o_i = frames[0:3, 3, i]
        #         z_i = frames[0:3, 2, i]  
        #         jacobians[0:3, i, j] = np.cross(z_i, o_j - o_i)
        #         jacobians[3:6, i, j] = z_i
            
        # return jacobians

        for i in range(self.dof):
            delta = np.zeros(len(thetas))
            delta[i] = epsilon 

            thetas_plus = thetas + delta 
            thetas_minus = thetas - delta 

            frames_plus = self.forward_kinematics_v1(thetas_plus)
            frames_minus = self.forward_kinematics_v1(thetas_minus)

            for j in range(self.dof + 1):
                position_plus = frames_plus[0:3, 3, j]
                position_minus = frames_minus[0:3, 3, j]
                delta_position = (position_plus - position_minus) / (2 * epsilon)

                rotation_plus = frames_plus[0:3, 0:3, j]
                rotation_minus = frames_minus[0:3, 0:3, j]
                diff_R = rotation_plus @ rotation_minus.T

                trace = np.trace(diff_R)

                cos_theta = (trace - 1)/2 
                cos_theta = np.clip(cos_theta, -1.0, 1.0)
                theta = np.arccos(cos_theta) 

                if abs(theta) < 1e-7:
                    delta_rotation = np.zeros(3)
                else:
                    axis_denominator = 2 * np.sin(theta)
                    if abs(axis_denominator) < 1e-7:
                        delta_rotation = np.zeros(3)
                    else:
                        axis = (1 / axis_denominator) * np.array([
                            diff_R[2, 1] - diff_R[1, 2],
                            diff_R[0, 2] - diff_R[2, 0],
                            diff_R[1, 0] - diff_R[0, 1]
                        ])
                        axis_norm = np.linalg.norm(axis)
                        if axis_norm < 1e-6:
                            delta_rotation = np.zeros(3)
                        else:
                            axis = axis / axis_norm
                            delta_rotation = theta * axis / (2*epsilon)

                # rotation_plus = frames_plus[0:3, 0:3, j]
                # rotation_minus = frames_minus[0:3, 0:3, j]
                # rotation_diff = rotation_plus @ rotation_minus.T

                # trace = np.trace(rotation_diff)
                # theta = np.arccos((trace - 1)/ 2)

                # axis = np.array([
                #     rotation_diff[2, 1] - rotation_diff[1, 2],
                #     rotation_diff[0, 2] - rotation_diff[2, 0],
                #     rotation_diff[1, 0] - rotation_diff[0, 1]
                #     ])
                # axis = axis/np.linalg.norm(axis)
                # delta_rotation = (theta * axis) / (2 * epsilon)    

                jacobians[0:3, i, j] = delta_position
                jacobians[3:6, i, j] = delta_rotation

        return jacobians

        # --------------- END STUDENT SECTION --------------------------------------------
    
    def _inverse_kinematics_v1(self, target_pose, seed_joints):
        """
        Compute inverse kinematics using Jacobian pseudo-inverse method.
        
        Your implementation should:
        1. Start from seed joints
        2. Iterate until convergence or max iterations
        3. Check joint limits and singularities
        4. Return None if no valid solution
        
        Parameters
        ----------
        target_pose : RigidTransform
            Desired end-effector pose
        seed_joints : np.ndarray
            Initial joint configuration
            
        Returns
        -------
        np.ndarray or None
            Joint angles that achieve target pose, or None if not found
            
        Hints
        -----
        - Use get_pose() and get_jacobian() from robot arm
        - Use _compute_rotation_error() for orientation error
        - Check joint limits with is_joints_reachable()
        - Track pose error magnitude for convergence
        - The iteration parameters are defined in RobotConfig and TaskConfig
        """
        # if seed_joints.shape[0] != (self.dof):
        #     raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        # #print(type(target_pose))    
        # #print(target_pose)
        # #print(target_pose.shape)
        # if target_pose.shape != (4,4):
        #     raise ValueError('Invalid target_pose: Expected a 4x4 Transformation.')
        
        # if seed_joints is None:
        #     seed_joints = self.robot.arm.get_joints()
        
        # # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # # TODO: Implement gradient inverse kinematics
        # max_iter = TaskConfig.IK_MAX_ITERATIONS
        # stop_gradient = TaskConfig.IK_TOLERANCE
        # gradient_step = 0.3

        # current_joints = seed_joints
        # current_pose = self.forward_kinematics(current_joints)[:,:,-1]

        # jacobian = self.jacobians(current_joints)[:,:,-1]
        # j_T = jacobian.T
        
        # gradient = j_T @ self._compute_config_error(current_pose,target_pose)
        # # print(f"Starting error: {self._compute_config_error(current_pose,target_pose)}")
        # # print(f"Starting gradient: {gradient}")

        # iter_count = 0
        # while(iter_count<max_iter and np.linalg.norm(gradient)>stop_gradient):
        #     current_joints -= gradient_step*gradient
        #     current_pose = self.forward_kinematics(current_joints)[:,:,-1]
        #     jacobian = self.jacobians(current_joints)[:,:,-1]
        #     j_T = jacobian.T
        #     gradient = j_T @ self._compute_config_error(current_pose,target_pose)
        #     iter_count += 1
        # #print(current_joints)
        # #print(iter_count)	
        # return current_joints

        max_iterations = TaskConfig.IK_MAX_ITERATIONS
        tolerance = TaskConfig.IK_TOLERANCE
        learning_rate = 0.01

        joint_angles = seed_joints.copy()
        for step in range(max_iterations):
            all_frames = self.forward_kinematics_v1(joint_angles)
            current_end_effector = all_frames[:, :, -1]
            position_delta = target_pose[:3, 3] - current_end_effector[:3, 3]
            desired_rotation = target_pose[:3, :3]
            current_rotation = current_end_effector[:3, :3]
            rotation_difference = desired_rotation @ current_rotation.T
            trace_value = np.trace(rotation_difference)
            cos_theta = (trace_value - 1) / 2.0
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            angular_error = np.arccos(cos_theta)

            if np.isclose(angular_error, 0):
                orientation_delta = np.zeros(3)
            else:
                orientation_delta = (angular_error / (2 * np.sin(angular_error))) * np.array([
                    rotation_difference[2, 1] - rotation_difference[1, 2],
                    rotation_difference[0, 2] - rotation_difference[2, 0],
                    rotation_difference[1, 0] - rotation_difference[0, 1]
                ])

            total_error = np.concatenate((position_delta, orientation_delta))
            error_norm = np.linalg.norm(total_error)

            if error_norm < tolerance:
                return joint_angles

            jacobian_matrices = self.jacobians_v1(joint_angles)
            jacobian = jacobian_matrices[:, :, -1]
            jacobian_pinv = np.linalg.pinv(jacobian, rcond=1e-6)

            delta_angles = learning_rate * jacobian_pinv @ total_error
            joint_angles += delta_angles
        print("didn't converge within max iters")
        return None












    
        # --------------- END STUDENT SECTION --------------------------------------------------

    def _get_transform(self,R,d):
        transform = np.array([
            [R[0,0],R[0,1],R[0,2],d[0]],
            [R[1,0],R[1,1],R[1,2],d[1]],
            [R[2,0],R[2,1],R[2,2],d[2]],
            [0,0,0,1]
        ])
        return transform
    
    def _get_dh_parameters(self, thetas):
        dh_matrix = np.array([
            [0, 0, 0.333, thetas[0]],
            [0, -np.pi/2, 0, thetas[1]],
            [0, np.pi/2, 0.316, thetas[2]],
            [0.0825, np.pi/2, 0, thetas[3]],
            [-0.0825, -np.pi/2, 0.384, thetas[4]],
            [0, np.pi/2, 0, thetas[5]],
            [0.088, np.pi/2, 0.107+0.1034, thetas[6]]
        ])
    
        return dh_matrix
    
    def _rotation_to_axis_angle(self,R):
        axis = np.cross(R[:,0], R[:,1])
        k = axis/np.linalg.norm(axis)
        theta = np.arccos((np.trace(R)-1) / 2)

        return k,theta
    
    def _axis_angle_to_rotation(self,axis,angle):
        x, y, z = axis
        theta = angle
        cos = np.cos(theta)
        sin = np.sin(theta)
        v = 1 - cos

        R = np.array([
            [cos + x*x*v, x*y*v - z*sin, x*z*v + y*sin],
            [y*x*v + z*sin, cos + y*y*v, y*z*v - x*sin],
            [z*x*v - y*sin, z*y*v + x*sin, cos + z*z*v]
        ])

        return R
    
    def _compute_config_error(self, current_pose, target_pose):
        current_pose = np.array(current_pose)
        target_pose = np.array(target_pose)

        current_translation = current_pose[:3,3]
        target_translation = target_pose[:3,3]
        diff_translation = current_translation - target_translation

        current_R = current_pose[:3,:3]
        target_R = target_pose[:3,:3]

        diff_R = current_R @ target_R.T

        trace = np.trace(diff_R)

        cos_theta = (trace - 1)/2 
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        theta = np.arccos(cos_theta) 

        if abs(theta) < 1e-7:
            diff_rotation = np.zeros(3)
        else:
            axis_denominator = 2 * np.sin(theta)
            if abs(axis_denominator) < 1e-7:
                diff_rotation = np.zeros(3)
            else:
                axis = (1 / axis_denominator) * np.array([
                    diff_R[2, 1] - diff_R[1, 2],
                    diff_R[0, 2] - diff_R[2, 0],
                    diff_R[1, 0] - diff_R[0, 1]
                ])
                axis_norm = np.linalg.norm(axis)
                if axis_norm < 1e-6:
                    diff_rotation = np.zeros(3)
                else:
                    axis = axis / axis_norm
                    diff_rotation = theta * axis

        return np.hstack((diff_translation, diff_rotation))


        # if np.linalg.norm(diff_R - np.eye(3)) < 1e-6:
        #     diff_rotation = np.zeros(3)
        #     return np.hstack((diff_translation,diff_rotation))

        # trace = np.trace(diff_R)
        # theta = np.arccos((trace-1)/ 2)
        # if np.linalg.norm(theta)<1e-6:
        #     diff_rotation = np.zeros(3)
        #     return np.hstack((diff_translation,diff_rotation))
        # axis = (1 / (2*np.sin(theta))) * np.array([diff_R[2,1] - diff_R[1,2], diff_R[0,2] - diff_R[2,0], diff_R[1,0] - diff_R[0,1]])
        
        # if np.linalg.norm(axis)<1e-6:
        #     diff_rotation = np.zeros(3)
        #     return np.hstack((diff_translation,diff_rotation))
        # else:
        #     diff_rotation = theta * axis
        #     return np.hstack((diff_translation,diff_rotation))

    def forward_kinematics_v2(self, dh_parameters, thetas):
        """
        Compute foward kinematics
        
        Your implementation should:
        1. Compute transformation matrices for each frame using DH parameters
        2. Compute end-effector pose
        
        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters (you can choose to apply the offset to the tool flange or the pen tip)
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            End-effector pose
        """
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        frames = np.zeros((4, 4, len(dh_parameters)+1))
        frames[:,:,0] = np.eye(4)
        # thetas = np.zeros(7)
    
        # For each joint
        for i in range(len(dh_parameters)):
            # Get DH parameters
            a = dh_parameters[i,0]      # Link length
            alpha = dh_parameters[i,1]   # Link twist
            d = dh_parameters[i,2]       # Link offset
    
            if i<7 :
                theta = thetas[i]            # Joint angle
            else : 
                theta = 0
            
            # Compute transformation matrix elements
            ct = np.cos(theta)
            st = np.sin(theta)
            ca = np.cos(alpha)
            sa = np.sin(alpha)
            
            # Build DH transformation matrix following updated convention from piazza
            Ti = np.array([
                [ct, -st, 0, a],
                [st*ca, ct*ca, -sa, -d*sa],
                [st*sa, ct*sa, ca, d*ca],
                [0, 0, 0, 1]
            ])
            
            # Update frame by multiplying with previous frame
            frames[:,:,i+1] = frames[:,:,i] @ Ti
            
        # for i in range(9):
        #     print("-----")
        #     print(i)
        #     print(frames[:,:,i])
        #     print("-----")
        
        # Return final transformation (end-effector pose)
        return frames[:,:,-1]
    def jacobians_v2(self, thetas):
        """
        Compute the Jacobians for each frame.

        Parameters
        ----------
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            Jacobians
        """
        if thetas.shape != (self.dof,):
            raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')

        jacobians = np.zeros((6, self.dof, self.dof + 1))
        epsilon = 1e-6

        # --------------- BEGIN STUDENT SECTION ----------------------------------------
        # TODO: Implement the numerical computation of the Jacobians

        # Hints:
        # - Perturb each joint angle by epsilon and compute the forward kinematics.
        # - Compute numerical derivatives for x, y, and positions.
        # - Determine the rotational component based on joint contributions.

        # if thetas.shape != (self.dof,):
        #     raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')

        # jacobians = np.zeros((6, self.dof, self.dof + 1))
        # frames = self.forward_kinematics(thetas)
        # for j in range(self.dof + 1):
        #     o_j = frames[0:3, 3, j]
        #     for i in range(self.dof):
        #         o_i = frames[0:3, 3, i]
        #         z_i = frames[0:3, 2, i]  
        #         jacobians[0:3, i, j] = np.cross(z_i, o_j - o_i)
        #         jacobians[3:6, i, j] = z_i
            
        # return jacobians

        for i in range(self.dof):
            delta = np.zeros(len(thetas))
            delta[i] = epsilon 

            thetas_plus = thetas + delta 
            thetas_minus = thetas - delta 

            frames_plus = self.forward_kinematics_v2(thetas_plus)
            frames_minus = self.forward_kinematics_v2(thetas_minus)

            for j in range(self.dof + 1):
                position_plus = frames_plus[0:3, 3, j]
                position_minus = frames_minus[0:3, 3, j]
                delta_position = (position_plus - position_minus) / (2 * epsilon)

                rotation_plus = frames_plus[0:3, 0:3, j]
                rotation_minus = frames_minus[0:3, 0:3, j]
                diff_R = rotation_plus @ rotation_minus.T

                trace = np.trace(diff_R)

                cos_theta = (trace - 1)/2 
                cos_theta = np.clip(cos_theta, -1.0, 1.0)
                theta = np.arccos(cos_theta) 

                if abs(theta) < 1e-7:
                    delta_rotation = np.zeros(3)
                else:
                    axis_denominator = 2 * np.sin(theta)
                    if abs(axis_denominator) < 1e-7:
                        delta_rotation = np.zeros(3)
                    else:
                        axis = (1 / axis_denominator) * np.array([
                            diff_R[2, 1] - diff_R[1, 2],
                            diff_R[0, 2] - diff_R[2, 0],
                            diff_R[1, 0] - diff_R[0, 1]
                        ])
                        axis_norm = np.linalg.norm(axis)
                        if axis_norm < 1e-6:
                            delta_rotation = np.zeros(3)
                        else:
                            axis = axis / axis_norm
                            delta_rotation = theta * axis / (2*epsilon)

                # rotation_plus = frames_plus[0:3, 0:3, j]
                # rotation_minus = frames_minus[0:3, 0:3, j]
                # rotation_diff = rotation_plus @ rotation_minus.T

                # trace = np.trace(rotation_diff)
                # theta = np.arccos((trace - 1)/ 2)

                # axis = np.array([
                #     rotation_diff[2, 1] - rotation_diff[1, 2],
                #     rotation_diff[0, 2] - rotation_diff[2, 0],
                #     rotation_diff[1, 0] - rotation_diff[0, 1]
                #     ])
                # axis = axis/np.linalg.norm(axis)
                # delta_rotation = (theta * axis) / (2 * epsilon)    

                jacobians[0:3, i, j] = delta_position
                jacobians[3:6, i, j] = delta_rotation

        return jacobians
    def _inverse_kinematics_v2(self, target_pose, seed_joints, dh_params):
        """
        Compute inverse kinematics using Jacobian pseudo-inverse method.

        Your implementation should:
        1. Start from seed joints
        2. Iterate until convergence or max iterations
        3. Check joint limits and singularities
        4. Return None if no valid solution

        Parameters
        ----------
        target_pose : 4x4 matrix
            Desired end-effector pose
        seed_joints : np.ndarray
            Initial joint configuration
            
        Returns
        -------
        np.ndarray or None
            Joint angles that achieve target pose, or None if not found
            
        Hints
        -----
        - Use get_pose() and get_jacobian() from robot arm
        - Use _compute_rotation_error() for orientation error
        - Check joint limits with is_joints_reachable()
        - Track pose error magnitude for convergence
        - The iteration parameters are defined in RobotConfig and TaskConfig
        """

        fa = FrankaArm()
        if seed_joints is None:
            seed_joints = fa.get_joints()
        if seed_joints.shape != (self.dof,):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        # if type(target_pose) != RigidTransform:
        #     raise ValueError('Invalid target_pose: Expected RigidTransform.')

        # Get iteration parameters
        max_iters = TaskConfig.IK_MAX_ITERATIONS
        convergence_threshold = TaskConfig.IK_TOLERANCE
        step_size = TaskConfig.PATH_RESOLUTION*4

        current_joints = seed_joints.copy()

        for iteration in range(max_iters):
            # Get current pose
            current_pose = self.forward_kinematics_v2(dh_params, current_joints)
            # print(current_pose)
            # print(type(current_pose))
            # input("hanging")
            
            # Compute errors
            position_error = target_pose[:3,3] - current_pose[:3,3]
            orientation_error = _compute_rotation_error(current_pose, target_pose)
            
            error = np.concatenate([position_error, orientation_error])
            error_magnitude = np.linalg.norm(error)
            
            # Check convergence
            if error_magnitude < convergence_threshold:
                if fa.is_joints_reachable(current_joints):
                    if d:
                        print("Converged!")
                    return current_joints
            
            # Get Jacobian
            J = fa.get_jacobian(current_joints)
            # print(J)
            # print(J.shape)
            
            # Check for singularity
            # svd_values = np.linalg.svd(J, compute_uv=False)
            # if np.min(svd_values) < 0.01:
            #     print("Near singularity")
            #     print(np.min(svd_values))
            #     return None

            J_pinv = np.linalg.pinv(J)
            
            # Compute joint update
            delta_q = J_pinv @ error
            
            # Limit step size
            if np.linalg.norm(delta_q) > step_size:
                delta_q = step_size * delta_q / np.linalg.norm(delta_q) 
            # '''TODO: you can, but you don't need to check step_size inside IK'''
            
            # Update joints
            new_joints = current_joints + delta_q
            
            # Check joint limits
            if not fa.is_joints_reachable(new_joints):
                new_joints = np.clip(
                    new_joints,
                    RobotConfig.JOINT_LIMITS_MIN*1.1,
                    RobotConfig.JOINT_LIMITS_MAX*0.9
                )
                if not fa.is_joints_reachable(new_joints):
                    print("unreachable new config")
                    return None
            
            current_joints = new_joints

        # Failed to converge within max iterations
        print("did not converge within max iter")
        return None
    
    def twist(self, eeFrame, target):
        R = eeFrame[:3, :3].T @ target[:3, :3]
        axis, angle = self.axis_angle(R)
        t = target[:3, 3] - eeFrame[:3, 3]
        v = eeFrame[:3, :3] @ (angle * axis)
        return np.concatenate((t, v))

    def numeric_jacobian(self, q, eps=1e-6):
        T = self.forward_kinematics(q)[:, :  -1]
        J = np.zeros((6,7))
        for i in range(7):
            qeps = q.copy()
            qeps[i] += eps
            Teps = self.forward_kinematics(qeps)[:, : -1]
            J[:,i] = self.twist(T, Teps) / eps
        return J
    
    def _inverse_kinematics_v3(self, target_pose, seed_joints):
        """
        Compute inverse kinematics using Jacobian pseudo-inverse method.
        
        Your implementation should:
        1. Start from seed joints
        2. Iterate until convergence or max iterations
        3. Check joint limits and singularities
        4. Return None if no valid solution
        
        Parameters
        ----------
        target_pose : RigidTransform
            Desired end-effector pose
        seed_joints : np.ndarray
            Initial joint configuration
            
        Returns
        -------
        np.ndarray or None
            Joint angles that achieve target pose, or None if not found
            
        Hints
        -----
        - Use get_pose() and get_jacobian() from robot arm
        - Use _compute_rotation_error() for orientation error
        - Check joint limits with is_joints_reachable()
        - Track pose error magnitude for convergence
        - The iteration parameters are defined in RobotConfig and TaskConfig
        """
        # if seed_joints.shape[0] != (self.dof):
        #     raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        # #print(type(target_pose))    
        # #print(target_pose)
        # #print(target_pose.shape)
        # if target_pose.shape != (4,4):
        #     raise ValueError('Invalid target_pose: Expected a 4x4 Transformation.')
        
        # if seed_joints is None:
        #     seed_joints = self.robot.arm.get_joints()
        
        # # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # # TODO: Implement gradient inverse kinematics
        # max_iter = TaskConfig.IK_MAX_ITERATIONS
        # stop_gradient = TaskConfig.IK_TOLERANCE
        # gradient_step = 0.3

        # current_joints = seed_joints
        # current_pose = self.forward_kinematics(current_joints)[:,:,-1]

        # jacobian = self.jacobians(current_joints)[:,:,-1]
        # j_T = jacobian.T
        
        # gradient = j_T @ self._compute_config_error(current_pose,target_pose)
        # # print(f"Starting error: {self._compute_config_error(current_pose,target_pose)}")
        # # print(f"Starting gradient: {gradient}")

        # iter_count = 0
        # while(iter_count<max_iter and np.linalg.norm(gradient)>stop_gradient):
        #     current_joints -= gradient_step*gradient
        #     current_pose = self.forward_kinematics(current_joints)[:,:,-1]
        #     jacobian = self.jacobians(current_joints)[:,:,-1]
        #     j_T = jacobian.T
        #     gradient = j_T @ self._compute_config_error(current_pose,target_pose)
        #     iter_count += 1
        # #print(current_joints)
        # #print(iter_count)	
        # return current_joints

        max_iterations = TaskConfig.IK_MAX_ITERATIONS
        tolerance = TaskConfig.IK_TOLERANCE
        learning_rate = 0.01

        joint_angles = seed_joints.copy()
        for step in range(max_iterations):
            all_frames = self.forward_kinematics_v1(joint_angles)
            current_end_effector = all_frames[:, :, -1]

            total_error = self.twist(current_end_effector, target_pose)
            error_norm = np.linalg.norm(total_error)

            if error_norm < tolerance:
                return joint_angles

            jacobian = self.numeric_jacobian(joint_angles)
            jacobian_pinv = np.linalg.pinv(jacobian, rcond=1e-6)

            delta_angles = learning_rate * jacobian_pinv @ total_error
            joint_angles += delta_angles
        print("didn't converge within max iters")
        return None