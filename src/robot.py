import sys
sys.path.append('../config')
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from robot_config import RobotConfig
from task_config import TaskConfig

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7
    
    def forward_kinematcis(self, dh_parameters, thetas):
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
    
        # For each joint
        for i in range(len(dh_parameters)):
            # Get DH parameters
            a = dh_parameters[i,0]      # Link length
            alpha = dh_parameters[i,1]   # Link twist
            d = dh_parameters[i,2]       # Link offset
            theta = thetas[i]            # Joint angle
            
            # Compute transformation matrix elements
            ct = np.cos(theta)
            st = np.sin(theta)
            ca = np.cos(alpha)
            sa = np.sin(alpha)
            
            # Build DH transformation matrix
            Ti = np.array([
                [ct, -st*ca, st*sa, a*ct],
                [st, ct*ca, -ct*sa, a*st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
            
            # Update frame by multiplying with previous frame
            frames[:,:,i+1] = frames[:,:,i] @ Ti
        
        # Return final transformation (end-effector pose)
        return frames[:,:,-1]
    
    def jacobians(self, thetas):
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
        epsilon = 0.001

        # For each frame
        for frame in range(self.dof + 1):
            # Get base pose for this frame
            base_pose = self.forward_kinematics(self.dh_params[:frame], thetas[:frame])[0] if frame > 0 else np.eye(4)
            base_pos = base_pose[:3,3]
            base_rot = base_pose[:3,:3]
            
            # For each joint affecting this frame
            for joint in range(min(frame, self.dof)):
                # Perturb joint angles
                thetas_perturbed = thetas.copy()
                thetas_perturbed[joint] += epsilon
                
                # Get perturbed pose
                perturbed_pose = self.forward_kinematics(self.dh_params[:frame], thetas_perturbed[:frame])[0] if frame > 0 else np.eye(4)
                perturbed_pos = perturbed_pose[:3,3]
                perturbed_rot = perturbed_pose[:3,:3]
                
                # Compute position Jacobian (linear velocity component)
                jacobians[:3,joint,frame] = (perturbed_pos - base_pos) / epsilon
                
                # Compute orientation Jacobian (angular velocity component)
                if frame > 0:
                    # Compute rotation difference
                    rot_diff = perturbed_rot @ base_rot.T
                    
                    # Convert to axis-angle representation
                    angle = np.arccos((np.trace(rot_diff) - 1) / 2)
                    
                    if abs(angle) < 1e-6:
                        jacobians[3:,joint,frame] = np.zeros(3)
                    else:
                        # Extract rotation axis
                        axis = np.array([
                            rot_diff[2,1] - rot_diff[1,2],
                            rot_diff[0,2] - rot_diff[2,0],
                            rot_diff[1,0] - rot_diff[0,1]
                        ])
                        axis = axis / (2 * np.sin(angle))
                        
                        # Angular velocity component
                        jacobians[3:,joint,frame] = (angle / epsilon) * axis
                else:
                    jacobians[3:,joint,frame] = np.zeros(3)

        return jacobians
    
    def _inverse_kinematics(self, target_pose, seed_joints):
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
        
        if seed_joints.shape != (self.dof):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        if type(target_pose) != RigidTransform:
            raise ValueError('Invalid target_pose: Expected RigidTransform.')
        
        if seed_joints is None:
            seed_joints = self.robot.arm.get_joints()
        
        # Get iteration parameters from RobotConfig
        max_iters = RobotConfig.MAX_IK_ITERATIONS
        convergence_threshold = RobotConfig.IK_CONVERGENCE_THRESHOLD
        step_size = RobotConfig.MAX_IK_STEP_SIZE
        
        current_joints = seed_joints.copy()
        
        for iteration in range(max_iters):
            # Get current pose using robot arm
            current_pose = self.robot.arm.get_pose()
            
            # Compute position error
            position_error = target_pose.translation - current_pose.translation
            
            # Compute orientation error using provided method
            orientation_error = self._compute_rotation_error(current_pose, target_pose)
            
            # Combine errors
            error = np.concatenate([position_error, orientation_error])
            error_magnitude = np.linalg.norm(error)
            
            # Check convergence
            if error_magnitude < convergence_threshold:
                if self.is_joints_reachable(current_joints):
                    return current_joints
                else:
                    return None
            
            # Get Jacobian from robot arm
            J = self.robot.arm.get_jacobian(current_joints)
            
            # Check for singularity
            svd_values = np.linalg.svd(J, compute_uv=False)
            if np.min(svd_values) < RobotConfig.SINGULARITY_THRESHOLD:
                return None
                
            # Compute joint update using damped pseudo-inverse
            J_T = J.T
            damping = RobotConfig.IK_DAMPING_FACTOR
            J_pinv = J_T @ np.linalg.inv(J @ J_T + damping * np.eye(6))
            delta_q = J_pinv @ error
            
            # Limit step size
            if np.linalg.norm(delta_q) > step_size:
                delta_q = step_size * delta_q / np.linalg.norm(delta_q)
                
            # Update joints
            new_joints = current_joints + delta_q
            
            # Check joint limits
            if not self.is_joints_reachable(new_joints):
                # Try to clamp to joint limits
                new_joints = np.clip(
                    new_joints,
                    RobotConfig.JOINT_LIMITS_LOWER,
                    RobotConfig.JOINT_LIMITS_UPPER
                )
                if not self.is_joints_reachable(new_joints):
                    return None
                    
            current_joints = new_joints
        
        # Failed to converge within max iterations
        return None

