package com.spamrobotics.util;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotCentricExt extends RobotCentric {

    final double[] xySpeeds = new double[2]; // x, y speeds in m/s

    public RobotCentricExt withFieldCentricSpeeds(double xSpeed, double ySpeed, double rotationSpeed, Rotation2d robotAngle) {
        xySpeeds[0] = xSpeed;
        xySpeeds[1] = ySpeed;
        Helpers.fromFieldRelativeSpeeds(xySpeeds, robotAngle);
        return withRobotCentricSpeeds(xySpeeds[0], xySpeeds[1], rotationSpeed);
    }

    public RobotCentricExt withRobotCentricSpeeds(double xSpeed, double ySpeed, double rotationSpeed) {
        VelocityX = xSpeed;
        VelocityY = ySpeed;
        RotationalRate = rotationSpeed;
        return this;
    }
}
