package frc.robot.commands;

import java.util.function.Function;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;

public class DriveToPose extends Command {
  
    private final DrivetrainSubsystem drivetrain;
    private final ChassisSpeeds noneSpeeds = new ChassisSpeeds(0, 0, 0);

    private Supplier<Pose2d> targetPoseSupplier = null;
    private Pose2d currentPose = null;
    private Pose2d targetPose = null;
    private boolean dynamicTarget = false;
    private PoseTarget poseTargetType = PoseTarget.STANDARD;
    private double maxSpeed = 1.0;
    private double maxAngularRate = DrivetrainSubsystem.MAX_ANGULAR_RATE;
    private Function<Pose2d, Pose2d> intermediatePoses = null;


    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        drivetrain = drivetrainSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose(DrivetrainSubsystem drivetrainSubsystem, Pose2d targetPose) {
        drivetrain = drivetrainSubsystem;
        this.targetPose = targetPose;
        addRequirements(drivetrainSubsystem);
    }

    public DriveToPose withDynamicTarget(boolean dynamicTarget) {
        this.dynamicTarget = dynamicTarget;
        return this;
    }

    public DriveToPose withPoseTargetType(PoseTarget target) {
        poseTargetType = target;
        return this;
    }

    public DriveToPose withMaxSpeed(double maxSpeedPercentage) {
        maxSpeed = maxSpeedPercentage;
        return this;
    }

    public DriveToPose withMaxAngularRate(double maxAngularRate) {
        this.maxAngularRate = maxAngularRate;
        return this;
    }

    public DriveToPose withIntermediatePoses(Function<Pose2d, Pose2d> intermediatePoses) {
        this.intermediatePoses = intermediatePoses;
        return this;
    }

    @Override
    public void initialize() {
        drivetrain.setPoseTargetType(poseTargetType);
        drivetrain.resetPIDs(HeadingTarget.POSE);

        if (targetPoseSupplier != null) {
            targetPose = targetPoseSupplier.get();
        }

        drivetrain.setTargetPose(targetPose);
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getPose();

        if (dynamicTarget) {
            if (targetPoseSupplier != null) targetPose = targetPoseSupplier.get();
        }
        drivetrain.setTargetPose(targetPose);
        Pose2d iterationTarget = targetPose;
        
        if (intermediatePoses != null) {
            iterationTarget = intermediatePoses.apply(targetPose);
        }
        drivetrain.setIntermediatePose(iterationTarget);

        final double maxSpeedMeters = DrivetrainSubsystem.MAX_SPEED * maxSpeed;
        drivetrain.setTargetPoseMaxSpeed(maxSpeedMeters);
  
        drivetrain.driveToStrategy.setMaxAngularRate(maxAngularRate);
        drivetrain.setControl(drivetrain.driveToStrategy.apply(
            drivetrain.closedLoopRobotCentric,
            currentPose,
            iterationTarget,
            Math.min(drivetrain.driveToPoseConstraints.maxVelocity, maxSpeedMeters)
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(noneSpeeds);
        drivetrain.setTargetPose(null);
        drivetrain.setIntermediatePose(null);
        drivetrain.setPoseTargetType(PoseTarget.STANDARD);
        drivetrain.setTargetHeading(null);
    }
}
