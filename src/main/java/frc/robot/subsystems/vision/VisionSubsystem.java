package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionIO.VisionInputs;

@Logged
public class VisionSubsystem extends SubsystemBase {

    public static final Transform3d TURRET_TO_CAMERA = new Transform3d(
        Inches.of(6.66233).in(Meters), // forward
        Inches.of(0).in(Meters),
        Inches.of(19.868824).in(Meters), // up
        new Rotation3d(0, Units.degreesToRadians(22), 0) // upward pitch
    );

    public static final Transform3d ROBOT_TO_MODULE_CAMERA = Transform3d.kZero;

    public static final int[] RED_HUB_TAG_IDS = { 9, 10, 8, 5, 4, 3, 2, 11 };
    public static final int[] BLUE_HUB_TAG_IDS = { 19, 20, 18, 27, 26, 25, 21, 24 };
    public static final int[] ALL_HUB_TAG_IDS;

    static {
        ALL_HUB_TAG_IDS = new int[RED_HUB_TAG_IDS.length + BLUE_HUB_TAG_IDS.length];
        System.arraycopy(RED_HUB_TAG_IDS, 0, ALL_HUB_TAG_IDS, 0, RED_HUB_TAG_IDS.length);
        System.arraycopy(BLUE_HUB_TAG_IDS, 0, ALL_HUB_TAG_IDS, RED_HUB_TAG_IDS.length, BLUE_HUB_TAG_IDS.length);
    }

    public static final Translation2d HUB_TARGET_TRANSLATION = new Translation2d(-0.2, 0); // Unused, for targeting back area of hub
    public static final Transform2d TAG_TO_HUB_CENTER_TRANSFORM = new Transform2d(-0.6, 0, Rotation2d.kZero);

    public static final Matrix<N3, N1> MT1_STDDEV = VecBuilder.fill(0.15, 0.15, 0.15);
    public static final Matrix<N3, N1> MT1_NO_HEADING_STDDEV = VecBuilder.fill(0.3, 0.3, 9999999);
    public static final Matrix<N3, N1> MT2_STDDEV = VecBuilder.fill(0.7,0.7,9999999);
    
    final VisionIO io;
    final VisionInputs inputs;
    final Pose2d blueHub, redHub;
    final Alert turretCameraDisconnectedAlert = new Alert("Turret Camera disconnected", AlertType.kError);

    PoseEstimate lastTurretPoseEstimate = null;
    double turretCameraLatencySeconds = 0;
    Pose2d turretPoseRobotEstimate = Pose2d.kZero;

    boolean odometryUpdatesEnabled = true;
    double estimateAmbiguity = 0;

    public VisionSubsystem() {
        io = Robot.isReal() ? new VisionIOLimelight() : new VisionIOPhoton();
        inputs = new VisionInputs();

        blueHub = Robot.aprilTagFieldLayout.getTagPose(26).get().toPose2d().transformBy(TAG_TO_HUB_CENTER_TRANSFORM);
        redHub = Robot.aprilTagFieldLayout.getTagPose(10).get().toPose2d().transformBy(TAG_TO_HUB_CENTER_TRANSFORM);
    }

    boolean usingMt2 = false;

    @Override
    public void periodic() {
        io.update(inputs);

        RobotContainer rc = RobotContainer.instance;
        PoseEstimate turretPoseEstimate = null;
    
        boolean mt2Allowed = Robot.isEnabledCached() && Robot.isAutonomousCached() && Robot.isReal();
        boolean mt1Allowed = !mt2Allowed;

        if (mt1Allowed) {
            turretPoseEstimate = validateTurretPoseEstimate(inputs.turretPoseEstimate, false);
            if (turretPoseEstimate != null) usingMt2 = false;
        }

        if (turretPoseEstimate == null && mt2Allowed) {
            turretPoseEstimate = validateTurretPoseEstimate(inputs.turretPoseEstimateMT2, true);
            usingMt2 = true;
        }

        if (turretPoseEstimate != null && (lastTurretPoseEstimate == null || turretPoseEstimate.timestampSeconds > lastTurretPoseEstimate.timestampSeconds)) {
            lastTurretPoseEstimate = turretPoseEstimate;
            turretCameraLatencySeconds = Timer.getFPGATimestamp() - turretPoseEstimate.timestampSeconds;

            // Compute latency-compensated pose estimate of the robot
            turretPoseRobotEstimate = TurretSubsystem.turretPoseToRobotPose(
                turretPoseEstimate.pose,
                rc.turret.getMeasuredAngleAt(turretPoseEstimate.timestampSeconds)
            );

            if (turretPoseEstimate.rawFiducials.length > 0 && turretPoseEstimate.rawFiducials[0] != null) {
                estimateAmbiguity = turretPoseEstimate.rawFiducials[0].ambiguity;
            }

            if (odometryUpdatesEnabled) {
                var stddev = usingMt2 ? MT2_STDDEV : MT1_STDDEV;
                if (!usingMt2 && Robot.isAutonomousCached()) stddev = MT1_NO_HEADING_STDDEV;
                rc.drivetrain.addVisionMeasurement(
                    turretPoseRobotEstimate,
                    turretPoseEstimate.timestampSeconds,
                    stddev
                );
            }
        } else {
            lastTurretPoseEstimate = null;
            turretPoseRobotEstimate = Pose2d.kZero;
        }

        turretCameraDisconnectedAlert.set(!inputs.turretCameraConnected);
    }

    @NotLogged
    public Pose2d getBlueHub() {
        return blueHub;
    }

    @NotLogged
    public Pose2d getRedHub() {
        return redHub;
    }

    @NotLogged
    public Pose2d getAllianceHub() {
        return Robot.isBlue() ? blueHub : redHub;
    }

    @NotLogged
    public Pose2d getOpponentAllianceHub() {
        return Robot.isBlue() ? redHub : blueHub;
    }

    @NotLogged
    public boolean isTurretCameraConnected() {
        return inputs.turretCameraConnected;
    }

    public void setOdometryUpdatesEnabled(boolean enabled) {
        odometryUpdatesEnabled = enabled;
    }

    public Command odometryUpdatesEnabled(boolean enabled) {
        return runOnce(() -> setOdometryUpdatesEnabled(enabled));
    }
    
    private boolean estimateWithinField(PoseEstimate estimate) {
        if (estimate.pose.getX() > Robot.aprilTagFieldLayout.getFieldLength() ||
            estimate.pose.getX() < 0 ||
            estimate.pose.getY() > Robot.aprilTagFieldLayout.getFieldWidth() ||
            estimate.pose.getY() < 0) {
            return false;
        }
        return true;
    }

    private PoseEstimate validatePoseEstimate(PoseEstimate estimate) {
        if (estimate == null || Robot.aprilTagFieldLayout == null) {
            return null;
        }

        if (estimate.tagCount < 1) {
            return null;
        }

        double maxDist = estimate.tagCount == 1 ? 4 : 6;
        if (estimate.avgTagDist > maxDist && Robot.isReal()) return null;

        if (Robot.isReal() && estimate.tagCount == 1 && estimate.rawFiducials[0] != null && estimate.rawFiducials[0].ambiguity > 0.5) return null;

        if (!estimateWithinField(estimate)) {
            return null;
        }

        return estimate;
    }

    private boolean almostZero(double num) {
        return Math.abs(num) <= 0.01;
    }

    public PoseEstimate validateMT2PoseEstimate(PoseEstimate estimate) {
        if (estimate == null || Robot.aprilTagFieldLayout == null) return null;
        if (estimate.tagCount < 1) return null;
        if (!estimateWithinField(estimate)) return null;

        // Invalid pose
        if (almostZero(estimate.pose.getX()) && almostZero(estimate.pose.getY())) {
            return null;
        }

        return estimate;
    }

    private PoseEstimate validateTurretPoseEstimate(PoseEstimate estimate, boolean mt2) {
        estimate = mt2 ? validateMT2PoseEstimate(estimate) : validatePoseEstimate(estimate);
        if (estimate == null) {
            return null;
        }

        TurretSubsystem turret = RobotContainer.instance.turret;
        if (Math.abs(turret.getVelocityRPS()) >= 0.5 || turret.isWrapping()) {
            return null;
        }

        return estimate;
    }

    public static AprilTagFieldLayout loadAprilTagFieldLayout() {
         try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
        } catch (Exception e) {
            return null;
        }
    }
}
