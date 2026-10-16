package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import frc.robot.LimelightHelpers.PoseEstimate;

@Logged
public interface VisionIO {

    @Logged
    public class VisionInputs {
        boolean turretCameraConnected = false;
        PoseEstimate turretPoseEstimate = null;
        PoseEstimate turretPoseEstimateMT2 = null;
    }

    public void update(VisionInputs inputs);
    
}
