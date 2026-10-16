package com.spamrobotics.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveStrategy {

    protected final double[] outputs = new double[3]; // x m/s, y m/s, angle rad/s

    public abstract void reset();
    protected abstract void calculateOutputs(Pose2d currentPose, Pose2d endPose, double maxVelocity);

    public ChassisSpeeds drive(Pose2d currentPose, Pose2d endPose, double maxVelocity) {
        calculateOutputs(currentPose, endPose, maxVelocity);
        return ChassisSpeeds.fromFieldRelativeSpeeds(outputs[0], outputs[1], outputs[2], currentPose.getRotation());
    }

    public SwerveRequest.RobotCentric apply(SwerveRequest.RobotCentric request, Pose2d currentPose, Pose2d endPose, double maxVelocity) {
        calculateOutputs(currentPose, endPose, maxVelocity);
        Helpers.fromFieldRelativeSpeeds(outputs, currentPose.getRotation());
        return request.withVelocityX(outputs[0])
                      .withVelocityY(outputs[1])
                      .withRotationalRate(outputs[2])
                      .withDriveRequestType(DriveRequestType.Velocity)
                      .withDesaturateWheelSpeeds(false);
    }
}