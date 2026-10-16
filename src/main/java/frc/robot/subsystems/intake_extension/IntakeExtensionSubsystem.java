package frc.robot.subsystems.intake_extension;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import com.spamrobotics.util.SubsystemExt;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

@Logged
public class IntakeExtensionSubsystem extends SubsystemExt {
    final IntakeExtensionIO io;
    final IntakeExtensionIO.Inputs inputs;

    private double currentTarget;
    @NotLogged
    private boolean zeroed = false;
    @NotLogged
    private boolean usingPid = false;
    private boolean commandedThisLoop = false;
    private boolean enteredCoast = false;

    private double currentMaxVelocity = MAX_VELOCITY;
    private double currentMaxAcceleration = MAX_ACCELERATION;

    public static final double MIN_POSITION = 0;
    public static final double MAX_POSITION = 2.967;

    public static final double TURRET_MIN_POSITION = 0.7;
    public static final double STOWED_POSITION = 0.756;
    public static final double EXTENDED_POSITION = 2.95;
    public static final double FUEL_JOSTLE_POSITION = STOWED_POSITION + 0.01;
    public static final double TOLERANCE = 0.05;

    public static final double MAX_ACCELERATION = 40 * 1.5;
    public static final double MAX_VELOCITY = 30;


    public enum Position {
        STOWED(STOWED_POSITION),
        EXTENDED(EXTENDED_POSITION),
        FUEL_JOSTLE(FUEL_JOSTLE_POSITION);

        private final double value;
        Position(double value) { this.value = value; }
        public double getValue() { return value; }
    }

    public IntakeExtensionSubsystem() {
        io = new IntakeExtensionIOTalonFX();
        inputs = new IntakeExtensionIO.Inputs();
        zeroed = true;
    }

    @Override
    public void periodic() {
        io.update(inputs);

        if (!Robot.isEnabledCached()) {
            currentTarget = clampPosition(getPosition());
            io.stop();
            return;
        }
    }

    @Override
    public void periodicAfterScheduler() {
        if (!usingPid) return;

        if (Robot.isAutonomousCached() || isTargeting(Position.STOWED)) commandedThisLoop = true;

        if (atTarget()) {
            io.coast();
            enteredCoast = true;
        } else if (commandedThisLoop || !enteredCoast) {
            io.setPosition(currentTarget, currentMaxVelocity, currentMaxAcceleration);
            enteredCoast = false;
        }
        // Reset flag for next loop
        commandedThisLoop = false;
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command startHome() {
        return runPower(-0.2);
    }

    public Command finishHome() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                io.resetPosition(0);
                zeroed = true;
            }),
            RobotContainer.instance.rumble(1).withTimeout(1)
        );
    }

    public Command coast() {
        return run(() -> io.coast());
    }

    public Command runPower(double power) {
        return runEnd(
            () -> {
                io.setPower(power);
                usingPid = false;
            },
            () -> io.setPower(0)
        );
    }

    public Command stowSlow() {
        return goToPosition(Position.STOWED, 0.8, MAX_ACCELERATION);
    }

    public Command jostleSlow() {
        return goToPosition(Position.FUEL_JOSTLE, 0.8, MAX_ACCELERATION);
    }

    public Command goToPosition(Position position) {
        return targetPosition(position).until(() -> atPosition(position));
    }

    public Command goToPosition(Position position, double maxVel, double maxAccel) {
        return targetPosition(position, maxVel, maxAccel).until(() -> atPosition(position));
    }

    public Command targetPosition(Position position) {
        return run(() -> setExtensionTarget(position));
    }

    public Command targetPosition(Position position, double maxVel, double maxAccel) {
        return run(() -> setExtensionTarget(position, maxVel, maxAccel));
    }

    public Command goToPositionAtPower(Position position, double power) {
        return runEnd(() -> {
            setExtensionTarget(position);
            usingPid = false;
            double sign = position.getValue() > inputs.position ? 1 : -1;
            io.setPower(power * sign);
        },
        () -> {
            io.setPower(0);
        })
        .until(() -> atPosition(position));
    }

    private void setExtensionTarget(Position position) {
        setExtensionTarget(position.getValue());
    }

    private void setExtensionTarget(Position position, double maxVel, double maxAccel) {
        setExtensionTarget(position.getValue(), maxVel, maxAccel);
    }

    private void setExtensionTarget(double position) {
        setExtensionTarget(position, MAX_VELOCITY, MAX_ACCELERATION);
    }

    private void setExtensionTarget(double position, double maxVel, double maxAccel) {
        currentTarget = clampPosition(position);
        currentMaxVelocity = maxVel;
        currentMaxAcceleration = maxAccel;
        commandedThisLoop = true;
        usingPid = true;
    }

    public boolean atTarget() {
        return atPosition(currentTarget);
    }

    public boolean atPosition(Position position) {
        return atPosition(position.getValue());
    }

    public boolean atPosition(double target) {
        return Math.abs(getPosition() - target) < TOLERANCE;
    }

    public boolean atPositionGenerous(double target) {
        return Math.abs(getPosition() - target) < TOLERANCE * 3;
    }

    public boolean isTargeting(double target) {
        return Math.abs(currentTarget - target) <= 0.001;
    }

    public boolean isTargeting(Position position) {
        return isTargeting(position.getValue());
    }

    public void stop() {
        currentTarget = clampPosition(getPosition());
        io.stop();
    }

    @NotLogged
    public double getPosition() {
        return inputs.position;
    }

    @NotLogged
    public double getTarget() {
        return currentTarget;
    }

    @NotLogged
    public Pose3d getVisPose() {
        double stowedPercent = (getPosition() - STOWED_POSITION) / (EXTENDED_POSITION - STOWED_POSITION);
        return new Pose3d(
            -0.2 + (stowedPercent * 0.2),
            0,
            0,
            Rotation3d.kZero
        );
    }

    private static double clampPosition(double position) {
        return Math.max(MIN_POSITION, Math.min(position, MAX_POSITION));
    }
}
