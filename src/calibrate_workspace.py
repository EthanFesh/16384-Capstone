from frankapy import FrankaArm
import numpy as np
from autolab_core import RigidTransform

class WorkspaceCalibrator:
    def __init__(self):
        self.fa = FrankaArm()
        self.duration = 12
        
    def calibrate_pen_holders(self):
        """Calibrate pen holder positions through guided movement"""
        print("\nCalibrating pen holder...")
        print("Moving to home position...")
        self.fa.reset_joints()
        self.fa.open_gripper()
        # self.fa.close_gripper()
        
        # Calibrating pen 1 (pink)
        input(f"Press Enter to calibrate pen 1 (pink)")
        print(f"Move robot above a pen, the position will be printed out after {self.duration} seconds")
        self.fa.run_guide_mode(duration=self.duration)  # Allow manual positioning
        
        # Record position
        current_pose = self.fa.get_pose()
        print(f"Recorded pen 1 at: {current_pose.translation}")
        # print(type(current_pose.translation))
        # print(current_pose.translation.shape)
        np.save("pink_pen_pose.npy", current_pose.translation)

        # Calibrating pen 2 (indigo)
        input(f"Press Enter to calibrate pen 2 (indigo)")
        print(f"Move robot above a pen, the position will be printed out after {self.duration} seconds")
        self.fa.run_guide_mode(duration=self.duration)  # Allow manual positioning
        
        # Record position
        current_pose = self.fa.get_pose()
        print(f"Recorded pen 2 at: {current_pose.translation}")
        # print(type(current_pose.translation))
        # print(current_pose.translation.shape)
        np.save("indigo_pen_pose.npy", current_pose.translation)

        # Calibrating pen 3 (green)
        input(f"Press Enter to calibrate pen 3 (green)")
        print(f"Move robot above a pen, the position will be printed out after {self.duration} seconds")
        self.fa.run_guide_mode(duration=self.duration)  # Allow manual positioning
        
        # Record position
        current_pose = self.fa.get_pose()
        print(f"Recorded pen 3 at: {current_pose.translation}")
        # print(type(current_pose.translation))
        # print(current_pose.translation.shape)
        np.save("green_pen_pose.npy", current_pose.translation)
        return current_pose
    
    def calibrate_whiteboard(self):
        """Calibrate whiteboard position and orientation"""
        print("\nCalibrating whiteboard...")
        print("Moving to home position...")
        self.fa.reset_joints()
        
        # Get three points to define whiteboard plane
        points = []
        for i in range(3):
            if i == 0:
                input(f"Press Enter to record whiteboard origin")
            else:
                input(f"Press Enter to record whiteboard point {i+1}")
            print(f"You can proceed after {self.duration} seconds")
            self.fa.run_guide_mode(duration=self.duration)
            current_pose = self.fa.get_pose()
            points.append(current_pose.translation)
            print(f"Recorded point {i+1} at {current_pose.translation}")
            
        # # Calculate whiteboard plane and orientation
        p1, p2, p3 = points
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)
        
        # Create whiteboard frame
        rotation = self._compute_orientation_matrix(normal)
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = p1
        print(f"Recorded whiteboard at: {T}")
        np.save("whiteboard_pose.npy", T)
        
        return current_pose
    
    def calibrate_drop_location(self):
        """Calibrate pen drop location"""
        print("\nCalibrating drop location...")
        print("Moving to home position...")
        self.fa.reset_joints()
        input("Press Enter to calibrate drop location")
        print(f"Move robot above the drop bin, the position will be printed out after {self.duration} seconds")
        self.fa.run_guide_mode(duration=self.duration)
        drop_pose = self.fa.get_pose()
        translation = drop_pose.translation
        rotation = drop_pose.rotation
        pose = np.eye(4)
        pose[:3, :3] = rotation
        pose[:3, 3] = translation
        print(f"Recorded drop bin at: {pose}")
        np.save("drop_bin_pose.npy", pose)
        return drop_pose
    
    def _compute_orientation_matrix(self, normal):
        """Compute rotation matrix from normal vector"""
        z_axis = normal
        x_axis = np.array([1, 0, 0])
        if abs(np.dot(z_axis, x_axis)) > 0.9:
            x_axis = np.array([0, 1, 0])
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        return np.column_stack((x_axis, y_axis, z_axis))

def main():
    calibrator = WorkspaceCalibrator()
    
    # Perform calibration
    # pen_positions = calibrator.calibrate_pen_holders()
    whiteboard_pose = calibrator.calibrate_whiteboard()
    # drop_pose = calibrator.calibrate_drop_location()

if __name__ == "__main__":
    main()