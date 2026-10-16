package com.spamrobotics.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Helpers {

    public static Command fastSequence(Command... commands) {
        return new FastSequenceCommand(commands);
    }

    public static Translation2d yFlip(Translation2d translation) {
        return new Translation2d(
            translation.getX(),
            Robot.FIELD_WIDTH - translation.getY()
        );
    }

    public static Translation2d allianceFlip(Translation2d translation) {
        return new Translation2d(
            Robot.FIELD_LENGTH - translation.getX(),
            Robot.FIELD_WIDTH - translation.getY()
        );
    }

    public static Pose2d allianceFlip(Pose2d pose) {
        return new Pose2d(
            Robot.FIELD_LENGTH - pose.getX(),
            Robot.FIELD_WIDTH - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.kPi)
        );
    }
    
    public static ChassisSpeeds addChassisSpeeds(ChassisSpeeds a, ChassisSpeeds b) {
        return new ChassisSpeeds(
            capValue(a.vxMetersPerSecond + b.vxMetersPerSecond, DrivetrainSubsystem.MAX_SPEED),
            capValue(a.vyMetersPerSecond + b.vyMetersPerSecond, DrivetrainSubsystem.MAX_SPEED),
            capValue(a.omegaRadiansPerSecond + b.omegaRadiansPerSecond, DrivetrainSubsystem.MAX_ANGULAR_RATE)
        );
    }

    public static ChassisSpeeds addChassisSpeedsOverwrite(ChassisSpeeds a, ChassisSpeeds b) {
        a.vxMetersPerSecond = capValue(a.vxMetersPerSecond + b.vxMetersPerSecond, DrivetrainSubsystem.MAX_SPEED);
        a.vyMetersPerSecond = capValue(a.vyMetersPerSecond + b.vyMetersPerSecond, DrivetrainSubsystem.MAX_SPEED);
        a.omegaRadiansPerSecond = capValue(a.omegaRadiansPerSecond + b.omegaRadiansPerSecond, DrivetrainSubsystem.MAX_ANGULAR_RATE);
        return a;
    }

    public static ChassisSpeeds rotateChassisSpeeds(ChassisSpeeds speeds, Rotation2d rotation) {
        Translation2d rotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond).rotateBy(rotation);
        return new ChassisSpeeds(rotated.getX(), rotated.getY(), speeds.omegaRadiansPerSecond);
    }

    public static double angleToPoint(Translation2d point, Translation2d vectorStart, Translation2d vectorEnd) {
        // Vector from start to end
        Translation2d vector = vectorEnd.minus(vectorStart);
        
        // Vector from start to point
        Translation2d toPoint = point.minus(vectorStart);
        
        // Calculate the dot product
        double dotProduct = vector.getX() * toPoint.getX() + vector.getY() * toPoint.getY();
        
        // Calculate the magnitudes of the vectors
        double vectorMagnitude = Math.sqrt(vector.getX() * vector.getX() + vector.getY() * vector.getY());
        double toPointMagnitude = Math.sqrt(toPoint.getX() * toPoint.getX() + toPoint.getY() * toPoint.getY());
        
        // Calculate the angle between the two vectors
        double angle = Math.acos(dotProduct / (vectorMagnitude * toPointMagnitude));
        
        // Convert the angle to degrees
        return Math.toDegrees(angle);
    }

    public static double capValue(double value, double max) {
        if (Math.abs(value) > max) return max * (value / Math.abs(value));
        return value;
    }

    public static void fromFieldRelativeSpeeds(double[] inputs, Rotation2d robotAngle) {
        rotateBy(inputs, -robotAngle.getRadians());
    }

    public static void rotateBy(double[] inputs, double radians) {
        if (inputs.length < 2) return;

        double x = inputs[0];
        double y = inputs[1];
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);

        inputs[0] = x * cos - y * sin;
        inputs[1] = x * sin + y * cos;
    }

    public static boolean withinTolerance(Pose2d pose, Pose2d targetPose, Distance xDistance, Distance yDistance, Angle angle) {
        return withinTolerance(
            pose,
            targetPose,
            xDistance == null ? 0 : xDistance.in(Meters),
            yDistance == null ? 0 : yDistance.in(Meters),
            angle == null ? 0 : angle.in(Degrees),
            xDistance == null,
            yDistance == null,
            angle == null
        );
    }

    public static boolean withinTolerance(Pose2d pose, Pose2d targetPose, double xMeters, double yMeters, double degrees,
        boolean skipX, boolean skipY, boolean skipDegrees) {
        // If one or more of the poses don't exist, we can't be within tolerance
        if (pose == null || targetPose == null) {
            return false;
        }
        
        boolean xSatisfied = skipX || Math.abs(pose.getX() - targetPose.getX()) <= xMeters;
        boolean ySatisfied = skipY || Math.abs(pose.getY() - targetPose.getY()) <= yMeters;

        // If we haven't met the x and y criteria, don't bother calculating any further
        if (!xSatisfied || !ySatisfied) {
            return false;
        }

        boolean headingSatisfied;
        if (skipDegrees) {
            headingSatisfied = true;
        } else {
            double poseDegrees = MathUtil.inputModulus(pose.getRotation().getDegrees(), -180, 180);
            double targetPoseDegrees = MathUtil.inputModulus(targetPose.getRotation().getDegrees(), -180, 180);
            double degreesDiff = MathUtil.inputModulus(poseDegrees - targetPoseDegrees, -180, 180);
            headingSatisfied = Math.abs(degreesDiff) <= degrees;
        }
        return headingSatisfied;
    }

    public static boolean poseWithinPOV(Pose2d robotPose, Pose2d targetPose, double fovDegrees, double detectionDistance) {
        // Define the triangle vertices based on the robot's pose
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation().rotateBy(Rotation2d.k180deg);

        // Define the field of view (FOV) angle and distance
        double fovAngle = Math.toRadians(fovDegrees);
        double fovDistance = detectionDistance;

        // Calculate the vertices of the triangle
        Translation2d vertex1 = robotTranslation;
        Translation2d vertex2 = robotTranslation.plus(new Translation2d(fovDistance, fovDistance * Math.tan(fovAngle / 2)).rotateBy(robotRotation));
        Translation2d vertex3 = robotTranslation.plus(new Translation2d(fovDistance, -fovDistance * Math.tan(fovAngle / 2)).rotateBy(robotRotation));

        // Check if the coralPose is within the triangle
        return isPointInTriangle(targetPose.getTranslation(), vertex1, vertex2, vertex3);
    }

    private static boolean isPointInTriangle(Translation2d pt, Translation2d v1, Translation2d v2, Translation2d v3) {
        double d1 = sign(pt, v1, v2);
        double d2 = sign(pt, v2, v3);
        double d3 = sign(pt, v3, v1);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }

    private static double sign(Translation2d p1, Translation2d p2, Translation2d p3) {
        return (p1.getX() - p3.getX()) * (p2.getY() - p3.getY()) - (p2.getX() - p3.getX()) * (p1.getY() - p3.getY());
    }

    // Numbers pulled from https://store.ctr-electronics.com/products/minion-brushless-motor
    public static DCMotor getMinion(int numMotors) {
        return new DCMotor(
            12,
            3.1,
            200.46,
            1.43,
            Units.rotationsPerMinuteToRadiansPerSecond(7200),
            numMotors
        );
    }

    // Numbers pulled from https://wcproducts.com/blogs/wcp-blog/kraken-x44
    public static DCMotor getKrakenX44(int numMotors) {
        return new DCMotor(
            12,
            4.05,
            275,
            1.4,
            Units.rotationsPerMinuteToRadiansPerSecond(7530),
            numMotors
        );
    }

    // Helper methods from https://github.com/frc6995/Robot-2025/blob/main/src/main/java/frc/robot/subsystems/drive/Pathing.java#L18

    public static double velocityTowards(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
        return pointRelativeSpeeds(currentPose, fieldRelativeSpeeds, targetTranslation).vxMetersPerSecond;
    }
    
    public static ChassisSpeeds pointRelativeSpeeds(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, headingTo(currentPose, targetTranslation));
    }
    
    public static Rotation2d headingTo(Pose2d currentPose, Translation2d target) {
        return target.minus(currentPose.getTranslation()).getAngle().minus(Rotation2d.kPi);
    }
}
