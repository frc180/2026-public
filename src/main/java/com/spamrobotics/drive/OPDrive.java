package com.spamrobotics.drive;

import com.spamrobotics.util.Helpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;

/**
 * Implements the driving strategy described in this post from 2056:
 * https://www.chiefdelphi.com/t/team-2056-op-robotics-2025-technical-binder-release/502550/36
 */
public class OPDrive extends DriveStrategy {

    final DrivetrainSubsystem drivetrain;
    final SlewRateLimiter rateLimiter;

    final PIDController translationPID = new PIDController(4.25, 0, 0.1); 

    boolean resetting = true;

    public OPDrive(DrivetrainSubsystem drivetrain, double rateLimitMeters) {
        this(drivetrain, rateLimitMeters, 999999);
    }

    public OPDrive(DrivetrainSubsystem drivetrain, double accelRateMPS, double decelRateMPS) {
        this.drivetrain = drivetrain;
        rateLimiter = new SlewRateLimiter(decelRateMPS, -accelRateMPS, 0.0);
        SmartDashboard.putData("AutoDrive PID", translationPID);
    }

    @Override
    public void reset() {
        resetting = true;
    }

    @Override
    protected void calculateOutputs(Pose2d currentPose, Pose2d endPose, double maxVelocity) {
        if (resetting) {
            translationPID.reset();
            drivetrain.resetHeadingPID(HeadingTarget.POSE);
            rateLimiter.reset(MathUtil.clamp(
                Helpers.velocityTowards(currentPose, drivetrain.getFieldRelativeSpeeds(), endPose.getTranslation()),
                -maxVelocity, 
                0
            ));

            resetting = false;
        }

        double normDirX = endPose.getX() - currentPose.getX();
        double normDirY = endPose.getY() - currentPose.getY();
        double distance = Math.hypot(normDirX, normDirY);

        normDirX /= (distance + 0.001);
        normDirY /= (distance + 0.001);


        double translationOutput = translationPID.calculate(distance, 0);
        translationOutput = rateLimiter.calculate(translationOutput);

        double xOutput = normDirX * -translationOutput;
        double yOutput = normDirY * -translationOutput;

        xOutput = MathUtil.clamp(xOutput, -maxVelocity, maxVelocity);
        yOutput = MathUtil.clamp(yOutput, -maxVelocity, maxVelocity);

        double thetaOutput = drivetrain.calculateHeadingPID(currentPose.getRotation(), endPose.getRotation().getDegrees());

        outputs[0] = xOutput;
        outputs[1] = yOutput;
        outputs[2] = thetaOutput;
    }
}
