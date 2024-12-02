import sys
sys.path.append('../config')
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from robot_config import RobotConfig
from task_config import TaskConfig
from utils import _compute_rotation_error

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7
    
    def forward_kinematics(self, dh_parameters, thetas):
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
        '''TODO: Add a final transformation from the flange to the center of grip or tip of pen'''
        # Return final transformation (end-effector pose)
        return frames[:,:,-1]
    
    def jacobians(self, thetas, dh_params):
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
            if frame > 0:
                # Use your working FK function
                base_frames = np.zeros((4, 4, len(dh_params)+1))
                base_frames[:,:,0] = np.eye(4)
                base_pose = self.forward_kinematics(dh_params, thetas)
                base_pos = base_pose[:3,3]
                base_rot = base_pose[:3,:3]
            else:
                base_pose = np.eye(4)
                base_pos = base_pose[:3,3]
                base_rot = base_pose[:3,:3]
            
            # For each joint affecting this frame
            for joint in range(min(frame, self.dof)):
                # Perturb joint angles
                thetas_perturbed = thetas.copy()
                thetas_perturbed[joint] += epsilon
                
                # Get perturbed pose
                if frame > 0:
                    perturbed_frames = np.zeros((4, 4, len(dh_params)+1))
                    perturbed_frames[:,:,0] = np.eye(4)
                    perturbed_pose = self.forward_kinematics(dh_params, thetas_perturbed)
                    perturbed_pos = perturbed_pose[:3,3]
                    perturbed_rot = perturbed_pose[:3,:3]
                else:
                    perturbed_pose = np.eye(4)
                    perturbed_pos = perturbed_pose[:3,3]
                    perturbed_rot = perturbed_pose[:3,:3]
                
                # Compute position Jacobian (linear velocity component)
                jacobians[:3,joint,frame] = (perturbed_pos - base_pos) / epsilon
                
                # Compute orientation Jacobian (angular velocity component)
                if frame > 0:
                    # Compute rotation difference
                    rot_diff = perturbed_rot @ base_rot.T
                    
                    # Convert to axis-angle representation
                    angle = np.arccos(np.clip((np.trace(rot_diff) - 1) / 2, -1.0, 1.0))
                    
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


    
    def _inverse_kinematics(self, target_pose, seed_joints, dh_params):
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
        step_size = TaskConfig.PATH_RESOLUTION
        
        current_joints = seed_joints.copy()
        
        for iteration in range(max_iters):
            # Get current pose
            current_pose = self.forward_kinematics(dh_params, current_joints)
            
            # Compute errors
            position_error = target_pose[:3,3] - current_pose[:3,3]
            orientation_error = _compute_rotation_error(current_pose, target_pose)
            
            error = np.concatenate([position_error, orientation_error])
            error_magnitude = np.linalg.norm(error)
            
            # Check convergence
            if error_magnitude < convergence_threshold:
                if fa.is_joints_reachable(current_joints):
                    print("Converged!")
                    return current_joints
            
            # Get Jacobian
            J = self.jacobians(current_joints, dh_params)
            # print(J)
            # print(J.shape)
            
            # Check for singularity
            svd_values = np.linalg.svd(J, compute_uv=False)
            if np.min(svd_values) < 0.1:
                print("Near singularity")
                return None

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
                    RobotConfig.JOINT_LIMITS_MIN,
                    RobotConfig.JOINT_LIMITS_MAX
                )
                if not fa.is_joints_reachable(new_joints):
                    print("unreachable new config")
                    return None
            
            current_joints = new_joints
        
        # Failed to converge within max iterations
        print("did not converge within max iter")
        return None