package frc.robot.subsystems.vision;

import com.spamrobotics.vision.LimelightStatus;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class VisionIOLimelight implements VisionIO {

    final String TURRET_LIMELIGHT = "limelight";

    final LimelightStatus turretLimelightStatus = new LimelightStatus(TURRET_LIMELIGHT);

    boolean isDisabled = false;
    boolean wasDisabled = false;

    @Override
    public void update(VisionInputs inputs) {
        isDisabled = Robot.isDisabledCached();

        // Disable these if doing Charuco board calibration
        if (isDisabled) {
            setLimelightPosition(TURRET_LIMELIGHT, VisionSubsystem.TURRET_TO_CAMERA);

            LimelightHelpers.SetFiducialIDFiltersOverride(TURRET_LIMELIGHT, VisionSubsystem.ALL_HUB_TAG_IDS);
        }
        setMT2Info();

        inputs.turretPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(TURRET_LIMELIGHT);
        inputs.turretPoseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(TURRET_LIMELIGHT);

        turretLimelightStatus.update();
        inputs.turretCameraConnected = turretLimelightStatus.isConnected();

        wasDisabled = isDisabled;
    }

    private void setMT2Info() {
        RobotContainer rc = RobotContainer.instance;

        if (isDisabled) {
            if (!wasDisabled) {
                LimelightHelpers.SetThrottle(TURRET_LIMELIGHT, 100);
            }
        } else {
            if (wasDisabled) {
                LimelightHelpers.SetThrottle(TURRET_LIMELIGHT, 0);
            }
        }
        
        double turretFieldVelocity = Units.rotationsToDegrees(rc.turret.getVelocityRPS()) + 
                                     Units.radiansToDegrees(rc.drivetrain.getCachedState().Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(
            TURRET_LIMELIGHT,
            rc.turret.getPose().getRotation().getDegrees(),
            turretFieldVelocity, 0, 0, 0, 0
        );
    }

    private void setLimelightPosition(String limelightName, Transform3d transform) {
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            transform.getX(), // + forward
            transform.getY(), // + right
            transform.getZ(), // + up
            Units.radiansToDegrees(transform.getRotation().getX()), // roll
            Units.radiansToDegrees(transform.getRotation().getY()), // pitch
            Units.radiansToDegrees(transform.getRotation().getZ()) // yaw
        );
    } 
}
