package frc.robot.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem.GoalTarget;
import frc.robot.util.ShotTables.ShotTable;

/**
 * A simple aim-on-the-move algorithm, based off of the one found here:
 * https://github.com/rr1706/2022-Main/blob/main/src/main/java/frc/robot/commands/TurretedShooter/SmartShooter.java
 */
@Logged
public class SimpleAOTM implements AimOnTheMove {

    private static final boolean DRAG_COMPENSATION_ENABLED = true;
    private static final double DRAG_K = 0.0232; 
    private static final double DRAG_SCALE = 1.0;

    @NotLogged
    private final int iterations;
    @NotLogged
    private final double phaseDelay = 0.03;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));

    int iterationsTaken = 0;
    double lastCalledTime = 0;
    double lastTurretAngle = 0;
    double lastHoodAngle = 0;
    double lastVx = 0;
    double lastVy = 0;
    boolean validLastValues = false;

    double distance = 0;
    double ax = 0;
    double ay = 0;
    boolean shotValid = false;

    public SimpleAOTM() {
        this(5);
    }

    public SimpleAOTM(int iterations) {
        this.iterations = iterations;
    }

    @Override
    public AimOnTheMoveResult adjust(Translation2d target, Pose2d robotPose, boolean actStationary) {
        double time = Timer.getFPGATimestamp();
        if (time - lastCalledTime > Constants.LOOP_TIME * 5) {
            validLastValues = false;
            System.out.println("AOTM: Invalidating last values due to time gap");
        }
        lastCalledTime = time;

        RobotContainer rc = RobotContainer.instance;
        boolean hubTarget = rc.turret.getGoalTarget() == GoalTarget.HUB;
        ShotTable shotTable = hubTarget ? ShotTables.getHubShotTable() : ShotTables.getFeedShotTable();

        Pose2d turretPose = rc.turret.getPose();
        ChassisSpeeds robotVel = rc.drivetrain.getFieldRelativeSpeeds();

        boolean moving = Math.abs(robotVel.vxMetersPerSecond) >= 0.03 || Math.abs(robotVel.vyMetersPerSecond) >= 0.03;
        Translation2d newTarget = new Translation2d(target.getX(), target.getY());

        if (moving && !actStationary) {
            // Borrowed from 6328 https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/a875de4792279070e4fcfe990adba8719ec91d5b/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java#L110-L118
            if (phaseDelay > 0) {
                // Project forward based on current velocity to account for processing delay
                ChassisSpeeds robotRelativeVel = rc.drivetrain.getCachedState().Speeds;
                robotPose =
                    robotPose.exp(
                        new Twist2d(
                            robotRelativeVel.vxMetersPerSecond * phaseDelay,
                            robotRelativeVel.vyMetersPerSecond * phaseDelay,
                            robotRelativeVel.omegaRadiansPerSecond * phaseDelay));
            }

            turretPose = rc.turret.robotPoseToTurretPose(robotPose);

            double distance = turretPose.getTranslation().getDistance(target);
            double timeOfFlight = shotTable.getTimeOfFlight(distance);
            double newTimeOfFlight;

            iterationsTaken = 0;
            for (int i = 0; i < iterations; i++) {
                iterationsTaken++;
                double x = target.getX() - dragAdjustedDisplacement(robotVel.vxMetersPerSecond, timeOfFlight);
                double y = target.getY() - dragAdjustedDisplacement(robotVel.vyMetersPerSecond, timeOfFlight);
                newTarget = new Translation2d(x, y);

                distance = turretPose.getTranslation().getDistance(newTarget);
                newTimeOfFlight = shotTable.getTimeOfFlight(distance);

                // Point has converged, can return early
                if(Math.abs(newTimeOfFlight - timeOfFlight) <= 0.01) {
                    break;
                }

                timeOfFlight = newTimeOfFlight;
            }
        }
        
        distance = turretPose.getTranslation().getDistance(newTarget);
        double turretAngleDegrees = rc.turret.calculateAngleToPoint(robotPose, newTarget.getX(), newTarget.getY());
        double hoodAngleDegrees = shotTable.getHoodAngle(distance);
        double flywheelRPS = shotTable.getFlywheelRPS(distance);

        if (!validLastValues) {
            lastTurretAngle = turretAngleDegrees;
            lastHoodAngle = hoodAngleDegrees;
            lastVx = robotVel.vxMetersPerSecond;
            lastVy = robotVel.vyMetersPerSecond;
            validLastValues = true;
        }

        double turretDeltaDegrees = Math.IEEEremainder(turretAngleDegrees - lastTurretAngle, 360.0);
        double turretVelocityRPS = Units.degreesToRotations(turretAngleFilter.calculate(turretDeltaDegrees / Constants.LOOP_TIME));
        double hoodVelocityRPS = Units.degreesToRotations(hoodAngleFilter.calculate((hoodAngleDegrees - lastHoodAngle) / Constants.LOOP_TIME));


        shotValid = hubTarget ? distance >= shotTable.minDistance && distance <= shotTable.maxDistance : true;


        lastTurretAngle = turretAngleDegrees;
        lastHoodAngle = hoodAngleDegrees;
        lastVx = robotVel.vxMetersPerSecond;
        lastVy = robotVel.vyMetersPerSecond;

        return new AimOnTheMoveResult(shotValid, newTarget, distance, turretAngleDegrees, turretVelocityRPS, hoodAngleDegrees, hoodVelocityRPS, flywheelRPS);
    }

    private static double dragAdjustedDisplacement(double v0, double timeOfFlight) {
        if (!DRAG_COMPENSATION_ENABLED) {
            return v0 * timeOfFlight;
        }
        double K = DRAG_K * DRAG_SCALE;
        return Math.signum(v0) / K * Math.log1p(K * Math.abs(v0) * timeOfFlight);
    }
}
