package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface AimOnTheMove {

    public AimOnTheMoveResult adjust(Translation2d target, Pose2d robotPose, boolean actStationary);

    public class AimOnTheMoveResult {
        public boolean shotValid;
        public Translation2d target;
        public double distanceToTarget;
        public double turretAngleDegrees;
        public double turretVelocityRPS;
        public double hoodAngleDegrees;
        public double hoodVelocityRPS;
        public double flywheelRPS;

        public AimOnTheMoveResult(
            boolean shotValid,
            Translation2d target,
            double distanceToTarget,
            double turretAngleDegrees,
            double turretVelocityRPS,
            double hoodAngleDegrees,
            double hoodVelocityRPS,
            double flywheelRPS) {
            this.shotValid = shotValid;
            this.target = target;
            this.distanceToTarget = distanceToTarget;
            this.turretAngleDegrees = turretAngleDegrees;
            this.turretVelocityRPS = turretVelocityRPS;
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.hoodVelocityRPS = hoodVelocityRPS;
            this.flywheelRPS = flywheelRPS;
        }
    }
}
