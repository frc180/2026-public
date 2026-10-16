package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.BaseStatusSignal;
import com.spamrobotics.util.IStatusSignalLoop;
import com.spamrobotics.util.SubsystemExt;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake_extension.IntakeExtensionSubsystem;
import frc.robot.subsystems.turret.TurretIO.FastTurretInputs;
import frc.robot.subsystems.turret.TurretIO.TurretInputs;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AimOnTheMove.AimOnTheMoveResult;
import frc.robot.util.ShotTables;
import frc.robot.util.SimpleAOTM;

@Logged
public class TurretSubsystem extends SubsystemExt implements IStatusSignalLoop {

    public enum GoalTarget {
        HUB,
        FEED
    }

    static final boolean REBOOT_REZERO_ALLOWED = false;

    static final double TURRET_GEAR = 104.0;
    static final double RATIO_A = TURRET_GEAR / 10.0;
    static final double RATIO_B = TURRET_GEAR / 11.0;

    static final double PHYSICAL_MAX_POSITION_DEGREES = 394;
    static final double PHSYICAL_MIN_POSITION_DEGREES = 13.9;

    static final double MAX_POSITION_DEGREES = PHYSICAL_MAX_POSITION_DEGREES - 5;
    static final double MIN_POSITION_DEGREES = PHSYICAL_MIN_POSITION_DEGREES + 5;

    // Expected value of combinedEncoderPosition when turret is at 0
    static final double EXPECTED_STARTING_DEGREES = 195.5;

    // Measured from getMeasuredAngle()'s value when this variable is set to 0
    static final double TURRET_ZERO_OFFSET_DEGREES = -164.5;

    public static final Transform2d ROBOT_TO_TURRET = new Transform2d(
        Inches.of(-3.9119).in(Meters),
        Inches.of(6.0301).in(Meters),
        Rotation2d.kZero // No rotation offset
    );
    static final Transform2d TURRET_TO_ROBOT = ROBOT_TO_TURRET.inverse();
    
    static final Translation3d VIS_TRANSFORM = new Translation3d(ROBOT_TO_TURRET.getX(), ROBOT_TO_TURRET.getY(), 0.32);

    public static final boolean ROBORIO_PROFILING = true;

    final TurretIO io;
    final TurretInputs inputs;
    final FastTurretInputs fastInputs;

    final TimeInterpolatableBuffer<Double> angleBuffer;
    final SimpleAOTM aimOnTheMove;

    final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        30, 15
    ));
    final State goalState = new State(0, 0);

    final Alert invalidTurretStart = new Alert("Turret not starting in valid position", AlertType.kError);
    final Alert rebootRezeroAlert = new Alert("Turret rebooted, auto-rezeroed!", AlertType.kWarning);

    GoalTarget goalTarget = GoalTarget.HUB;
    AimOnTheMoveResult lastAOTMResult = null;

    Translation2d baseTarget = null;

    // Range set by the last setAngle() call, -180 to 180 degrees
    double requestedAngleDegrees = 0.0;
    // Actual absolute angle target sent to the motor controller (within 0 to 720 degrees)
    double targetAbsoluteAngleDegrees = 0.0;
    // Distance from turret to current goal (hub or feed), with SOTM compensation
    double distanceToTargetMeters = 0;
    // Distance from turret to the hub, with no SOTM compensation
    double hubDistanceMeters;
    double combinedEncoderPosition = 0;

    double targetVelocity = 0;
    State setpoint = new State(Units.degreesToRotations(180), 0);

    double toleranceDeg = 0;

    boolean isWrapping = false;
    boolean firstUpdate = true;
    boolean firstDisabled = true;
    boolean zeroed = false;
    boolean pidTargeting = false;
    boolean enabledSoftwareLimits = false;
    boolean skippingMotionProfile = false;

    Pose2d position = Pose2d.kZero;
    Pose2d simPosition = Pose2d.kZero;
    Pose3d goal = Pose3d.kZero;
    

    public TurretSubsystem() {
        io = new TurretIOTalonFX();
        inputs = new TurretInputs();
        fastInputs = new FastTurretInputs();
        angleBuffer = TimeInterpolatableBuffer.createDoubleBuffer(2);
        aimOnTheMove = new SimpleAOTM(10);
    }

    // debug
    double absAStartDegrees = 0;
    double absBStartDegrees = 0;
    double motorStartDegrees = 0;

    double t_absARelative = 0;
    double t_absBRelative = 0;
    double t_motorRelative = 0;

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return io.getStatusSignals();
    }

    @Override
    public void onLoop() {
        io.fastUpdate(fastInputs);
        angleBuffer.addSample(Timer.getFPGATimestamp(), getMeasuredAngle());
    }

    private double capTo360(double degrees) {
        while (degrees < 0) degrees += 360;
        return degrees % 360;
    }

    @Override
    public void periodic() {
        io.update(inputs);

        combinedEncoderPosition = getAbsolutePosition(
            capTo360(inputs.absolutePositionDegrees),
            capTo360(inputs.secondaryAbsolutePositionDegrees % 360)
        );

        if (Robot.isDisabledCached()) {
            setpoint = new State(Units.degreesToRotations(fastInputs.positionDegrees), 0.0);
            io.coast();
        }

        if (!zeroed || (Robot.isDisabledCached() && firstDisabled)) {
            boolean combinedEncoderPositionValid = Math.abs(combinedEncoderPosition - EXPECTED_STARTING_DEGREES) < 25;
            if (Robot.isReal()) {
                if (combinedEncoderPositionValid && Math.abs(fastInputs.velocityRPS) < 0.01) {
                    io.resetPosition(combinedEncoderPosition);
                    zeroed = true;
                };
                invalidTurretStart.set(!zeroed);
            } else {
                zeroed = true;
            }
        }

        if (Robot.isDisabledCached() && firstDisabled) {
            if (firstUpdate) {
                absAStartDegrees = inputs.absolutePositionDegrees;
                absBStartDegrees = inputs.secondaryAbsolutePositionDegrees;
                motorStartDegrees = fastInputs.positionDegrees;
            }

            firstUpdate = false;
        } else {
            firstDisabled = true;
        }

        if (!zeroed && REBOOT_REZERO_ALLOWED) {
            if (fastInputs.positionDegrees >= MIN_POSITION_DEGREES && fastInputs.positionDegrees <= MAX_POSITION_DEGREES) {
                zeroed = true;
                rebootRezeroAlert.set(true);
            }
        }

        if (zeroed && !enabledSoftwareLimits) {
            if (io.enableSoftwareLimits()) {
                enabledSoftwareLimits = true;
            }
        }

        t_absARelative = (inputs.absolutePositionDegrees - absAStartDegrees) / RATIO_A;
        t_absBRelative = (inputs.secondaryAbsolutePositionDegrees - absBStartDegrees) / RATIO_B;
        t_motorRelative = (fastInputs.positionDegrees - motorStartDegrees);
        

        RobotContainer rc = RobotContainer.instance;
        position = robotPoseToTurretPose(rc.drivetrain.getPose());
        if (Robot.isSimulation()) {
            simPosition = robotPoseToTurretPose(rc.drivetrain.getSimPose());
        }
        
        if (Robot.isAutonomousCached() || rc.inAllianceZone.getAsBoolean()) {
            goalTarget = GoalTarget.HUB;
        } else {
            goalTarget = GoalTarget.FEED;
        }

        hubDistanceMeters = position.getTranslation().getDistance(rc.vision.getAllianceHub().getTranslation());
    }

    private boolean safeToPid() {
        return zeroed && RobotContainer.instance.intakeExtension.getPosition() >= IntakeExtensionSubsystem.TURRET_MIN_POSITION;
    }

    double profiledSetpointDegrees = 0;
    double profiledSetpointVelocity = 0;

    // Shot table tuning only
    double rpmLookup = 0;
    double hoodLookup = 0;

    @Override
    public void periodicAfterScheduler() {
        RobotContainer rc = RobotContainer.instance; 
        // If there is a target position, actively track it
        if (baseTarget != null) {
            Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();

            // A little hack to prevent time loss by aiming while out of zone in auto
            boolean autoHack = Robot.isAutonomousCached() && !rc.inAllianceZone.getAsBoolean();
            lastAOTMResult = aimOnTheMove.adjust(baseTarget, robotPose, autoHack);

            Translation2d target = lastAOTMResult.target;
            distanceToTargetMeters = lastAOTMResult.distanceToTarget;
            goal = new Pose3d(target.getX(), target.getY(), 2, Rotation3d.kZero); // Visualization only

            setTargetAngle(lastAOTMResult.turretAngleDegrees, lastAOTMResult.turretVelocityRPS);

            // Shot table tuning only
            var table = ShotTables.getHubShotTable();
            rpmLookup = table.getFlywheelRPS(distanceToTargetMeters);
            hoodLookup = table.getHoodAngle(distanceToTargetMeters);
        }

        isWrapping = Math.abs(targetAbsoluteAngleDegrees - fastInputs.positionDegrees) > 180;

        if (!pidTargeting || !safeToPid()) return;

        double robotRelativeGoalVelocity = targetVelocity - Units.radiansToRotations(rc.drivetrain.getCachedState().Speeds.omegaRadiansPerSecond);

        if (!ROBORIO_PROFILING) {
            io.setPositionTarget(targetAbsoluteAngleDegrees, robotRelativeGoalVelocity);
            return;
        }

        // If we're close to the target, just go there directly to minimize response time
        if (Math.abs(fastInputs.positionDegrees - targetAbsoluteAngleDegrees) < 20) {
            io.setPositionTarget(targetAbsoluteAngleDegrees, robotRelativeGoalVelocity);
            skippingMotionProfile = true;
            return;
        }

        // If we were previously skipping the motion profile, we need to initialize it with the current position and velocity to prevent a discontinuity
        if (skippingMotionProfile) {
            setpoint.position = Units.degreesToRotations(fastInputs.positionDegrees);
            setpoint.velocity = fastInputs.velocityRPS;
            skippingMotionProfile = false;
        }

        goalState.position = Units.degreesToRotations(targetAbsoluteAngleDegrees);
        goalState.velocity = robotRelativeGoalVelocity;
        setpoint = profile.calculate(Constants.LOOP_TIME, setpoint, goalState);
        profiledSetpointDegrees = Units.rotationsToDegrees(setpoint.position);
        profiledSetpointVelocity = setpoint.velocity;

        io.setPositionTarget(profiledSetpointDegrees, robotRelativeGoalVelocity); // Does not pass motion profile feedforward
    }

    public Command t_copyTableToTuning() {
        return Commands.runOnce(() -> {
            SmartDashboard.putNumber("Hood Degrees", hoodLookup);
            SmartDashboard.putNumber("Shooter RPS", rpmLookup);
        }).ignoringDisable(true);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    @NotLogged
    public GoalTarget getGoalTarget() {
        return goalTarget;
    }

    @NotLogged
    public boolean isTargetingHub() {
        return goalTarget == GoalTarget.HUB;
    }

    @NotLogged
    public boolean isTargetingFeed() {
        return goalTarget == GoalTarget.FEED;
    }

    /**
     * Gets the current turret angle normalized to [-180, 180], with 0 being facing forward.
     * Useful for vision calculations or driver feedback.
     * @return The current angle in degrees within [-180, 180].
     */
    public double getMeasuredAngle() {
        return normalizeAngle(fastInputs.positionDegrees - TURRET_ZERO_OFFSET_DEGREES);
    }

    public double getMeasuredAngleAt(double timestamp) {
        Optional<Double> angleSample = angleBuffer.getSample(timestamp);
        if (angleSample.isEmpty()) {
            return 0;
        }
        return angleSample.get();
    }

    @NotLogged
    public double getVelocityRPS() {
        return fastInputs.velocityRPS;
    }

    @NotLogged
    public double getDistanceToTargetMeters() {
        return distanceToTargetMeters;
    }

    @NotLogged
    public double getSimpleDistanceToHub() {
        return hubDistanceMeters;
    }

    @NotLogged
    public AimOnTheMoveResult getLastAOTMResult() {
        return lastAOTMResult;
    }

    public boolean isShotValid() {
        return lastAOTMResult == null || lastAOTMResult.shotValid;
    }

    @NotLogged
    public boolean isWrapping() {
        return isWrapping;
    }

    @NotLogged
    public boolean isZeroed() {
        return zeroed;
    }

    @NotLogged
    public Pose2d getPose() {
        return position;
    }

    @NotLogged
    public Pose2d getSimPose() {
        return Robot.isSimulation() ? simPosition : position;
    }

    @NotLogged
    public Pose3d getVisPose() {
        return new Pose3d(
            VIS_TRANSFORM,
            new Rotation3d(0, 0, Math.toRadians(-getMeasuredAngle() - 90))
        );
    }

    public Command holdAngle() {
        return run(() -> {
            pidTargeting = false;
            baseTarget = null;
            targetAbsoluteAngleDegrees = fastInputs.positionDegrees;
            io.setSpeed(0);
        });
    }

    public Command runControlled(DoubleSupplier axis) {
        return runEnd(
            () -> {
                pidTargeting = false;
                baseTarget = null;
                io.setSpeed(axis.getAsDouble());
            },
            () -> io.setSpeed(0)
        );
    }
    
    public Command runSpeed(double power) {
        return runEnd(
            () -> {
                pidTargeting = false;
                baseTarget = null;
                io.setSpeed(power);
            },
            () -> io.setSpeed(0)
        );
    }

    /**
     * Sets the turret to a specific angle in degrees.
     * @param degrees The target angle in degrees.
     */
    public Command runAngle(double degrees) {
        return run(() -> {
            baseTarget = null;
            setTargetAngle(degrees);
        });
    }

    public Command defaultCommand() {
        return run(() -> {
            VisionSubsystem vision = RobotContainer.instance.vision;
            if (vision.isTurretCameraConnected()) {
                Translation2d position;
                if (RobotContainer.instance.isInAllianceZone(!Robot.isBlue())) {
                    // If we're in the opponent's zone, point at their hub to get better vision data
                    position = vision.getOpponentAllianceHub().getTranslation();
                } else {
                    position = vision.getAllianceHub().getTranslation();
                }
                setTargetPosition(position.getX(), position.getY());
            } else {
                baseTarget = null;
                setTargetAngle(0);
            }
        });
    }

    public Command pointAt(Supplier<Translation2d> positionSupplier) {
        return run(() -> {
            Translation2d position = positionSupplier.get();
            setTargetPosition(position.getX(), position.getY());
        });
    }

    public Command pointAt(Translation2d position) {
        return pointAt(position.getX(), position.getY());
    }

    public Command pointAt(double x, double y) {
        return run(() -> setTargetPosition(x, y));
    }

    public boolean atTarget() {
        if (lastAOTMResult != null && goalTarget == GoalTarget.HUB) {
            double dynamicToleranceRad = Math.atan(0.61 / lastAOTMResult.distanceToTarget);
            toleranceDeg = Units.radiansToDegrees(dynamicToleranceRad);
        } else {
            toleranceDeg = goalTarget == GoalTarget.HUB ? 5 : 7.5;
        }
        return getTargetError() < toleranceDeg;
    }

    @NotLogged
    public double getTargetError() {
        return Math.abs(fastInputs.positionDegrees - targetAbsoluteAngleDegrees);
    }

    public double calculateAngleToPoint(Pose2d robotPose, double targetX, double targetY) {
        double deltaX = targetX - position.getX();
        double deltaY = targetY - position.getY();
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        return -normalizeAngle(angleToTarget - robotPose.getRotation().getDegrees());
    }

    private void setTargetPosition(double targetX, double targetY) {
        pidTargeting = true;
        baseTarget = new Translation2d(targetX, targetY);
    }

    private void setTargetAngle(double degrees) {
        setTargetAngle(degrees, 0);
    }

    private void setTargetAngle(double degrees, double velocityRPS) {
        pidTargeting = true;
        requestedAngleDegrees = normalizeAngle(degrees);
        targetAbsoluteAngleDegrees = findClosestEquivalentAngle(
            fastInputs.positionDegrees,
            normalizeAngle(requestedAngleDegrees + TURRET_ZERO_OFFSET_DEGREES)
        );
        targetVelocity = velocityRPS;
    }

    public Pose2d robotPoseToTurretPose(Pose2d robotPose) {
        Pose2d offsetPose = robotPose.transformBy(ROBOT_TO_TURRET);
        return new Pose2d(
            offsetPose.getTranslation(),
            Rotation2d.fromDegrees(-getMeasuredAngle() + offsetPose.getRotation().getDegrees())
        );
    }

    public static Pose2d turretPoseToRobotPose(Pose2d turretPose, double turretAngleDegrees) {
        Pose2d offsetPose = new Pose2d(
            turretPose.getTranslation(),
            Rotation2d.fromDegrees(turretPose.getRotation().getDegrees() + turretAngleDegrees)
        );
        return offsetPose.transformBy(TURRET_TO_ROBOT);
    }

    /**
     * Helper to normalize any angle to [-180, 180].
     * @param angleDegrees Angle to normalize
     * @return Normalized angle
     */
    static double normalizeAngle(double angleDegrees) {
        double angle = angleDegrees % 360.0;
        angle = (angle + 360) % 360.0;
        if (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    /**
     * Finds the closest equivalent angle to the target that is within the turret's range of motion.
     * 
     * @param currentPosition The current position of the turret in degrees
     * @param targetDegrees The desired angle in the range [-180, 180]
     * @return The closest equivalent angle within [MIN_POSITION, MAX_POSITION]
     */
    static double findClosestEquivalentAngle(double currentPosition, double targetDegrees) {
        // Normalize target to [0, 360) range
        double normalizedTarget = ((targetDegrees % 360) + 360) % 360;
        
        // Calculate how many full rotations are needed to cover the range
        int minRotations = (int) Math.floor(MIN_POSITION_DEGREES / 360.0);
        int maxRotations = (int) Math.ceil(MAX_POSITION_DEGREES / 360.0);
        
        // Find all equivalent angles within the valid range
        double closestAngle = normalizedTarget;
        double minDistance = Double.MAX_VALUE;
        
        // Check equivalent angles at 360-degree intervals
        for (int rotations = minRotations; rotations <= maxRotations; rotations++) {
            double equivalentAngle = normalizedTarget + (rotations * 360);
            
            // Check if this angle is within the valid range
            if (equivalentAngle >= MIN_POSITION_DEGREES && equivalentAngle <= MAX_POSITION_DEGREES) {
                double distance = Math.abs(currentPosition - equivalentAngle);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestAngle = equivalentAngle;
                }
            }
        }
        
        // Clamp to valid range as a safety measure
        return Math.max(MIN_POSITION_DEGREES, Math.min(MAX_POSITION_DEGREES, closestAngle));
    }

    /**
     * Calculates the turret position based on the two encoder readings.
     * @param readingA Current angle of Encoder A in degrees (0-360)
     * @param readingB Current angle of Encoder B in degrees (0-360)
     * @return The calculated turret angle (0-360)
     */
    private double getAbsolutePosition(double readingA, double readingB) {
        double ratioDifference = Math.abs(RATIO_A - RATIO_B);
        double diff;
        
        // Always subtract the Slower Encoder from the Faster Encoder
        // to ensure the math moves in the positive direction.
        if (RATIO_A > RATIO_B) {
             // A is faster (e.g., 11 vs 10)
            diff = readingA - readingB;
        } else {
             // B is faster (e.g., 10 vs 11)
            diff = readingB - readingA;
        }

        double wrappedDiff = floorModDouble(diff, 360.0);

        // Scale by the difference in ratios.
        return wrappedDiff / ratioDifference;
    }

    // Helper to handle modulo for negative numbers like Python/Excel do
    // (Java's native '%' operator returns negative values for negative inputs)
    private double floorModDouble(double value, double modulus) {
        return ((value % modulus) + modulus) % modulus;
    }
}
