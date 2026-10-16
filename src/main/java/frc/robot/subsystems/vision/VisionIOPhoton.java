package frc.robot.subsystems.vision;

import com.spamrobotics.vision.SimCamera;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionIOPhoton implements VisionIO {

    final SimCamera turretCamera;

    public VisionIOPhoton() {
        turretCamera = new SimCamera(
            "TurretCamera",
            () -> RobotContainer.instance.turret.getSimPose(),
            VisionSubsystem.TURRET_TO_CAMERA,
            Robot.aprilTagFieldLayout
        );
    }

    @Override
    public void update(VisionInputs inputs) {
        if (Robot.isSimulation()) {
            turretCamera.update();
            PoseEstimate estimate = turretCamera.getPoseEstimate();
            inputs.turretPoseEstimate = estimate;
            inputs.turretPoseEstimateMT2 = estimate;
            inputs.turretCameraConnected = true;
        }
    }
}
