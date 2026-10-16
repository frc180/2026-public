package frc.robot.subsystems.hood;

import com.spamrobotics.util.SubsystemExt;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.HoodIO.HoodInputs;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ShotTables;
import frc.robot.util.AimOnTheMove.AimOnTheMoveResult;

@Logged
public class HoodSubystem extends SubsystemExt {

    public static final double MIN_DEGREES = -5.888672;
    public static final double MAX_DEGREES = Robot.isReal() ? 36.8 : 100;

    public static final double START_DEGREES = MIN_DEGREES;

    public static final double TARGET_ERROR_TOLERANCE = 1;

    public static final boolean ROBORIO_PROFILING = false;

    private final HoodIO io;
    private final HoodInputs inputs;

    final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(999, 25));
    final State goalState = new State(0, 0);

    State setpoint = new State(0, 0);

    double targetPositionDegrees = 0;
    double targetVelocityRPS = 0;
    boolean firstDisable = true;
    boolean pidControl = false;

    public HoodSubystem() {
        io = new HoodIOTalonFX();
        inputs = new HoodInputs();

        io.resetAngle(START_DEGREES);
    }

    @Override
    public void periodic() {
        io.update(inputs);

        if (Robot.isDisabledCached()) {
            setpoint.position = Units.degreesToRotations(inputs.positionDegrees);
            setpoint.velocity = 0;
        } else {
            firstDisable = false;
        }
    }

    @Override
    public void periodicAfterScheduler() {
        if (!pidControl) return;

        if (!ROBORIO_PROFILING) {
            io.setAngle(targetPositionDegrees, targetVelocityRPS);
            return;
        }

        goalState.position = Units.degreesToRotations(targetPositionDegrees);
        goalState.velocity = targetVelocityRPS;
        setpoint = profile.calculate(Constants.LOOP_TIME, setpoint, goalState);
        io.setAngle(Units.rotationsToDegrees(setpoint.position), setpoint.velocity);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    @NotLogged
    public double getDegrees() {
        return inputs.positionDegrees;
    }

    public boolean atTarget() {
        return Math.abs(targetPositionDegrees - getDegrees()) <= TARGET_ERROR_TOLERANCE;
    }

    public boolean atStowed() {
        return Math.abs(MIN_DEGREES - getDegrees()) <= TARGET_ERROR_TOLERANCE;
    }

    public Command rezero() {
        return Commands.runOnce(() -> io.resetAngleNonBlocking(START_DEGREES));
    }

    public Command runPower(double power) {
        return runEnd(
            () -> {
                pidControl = false;
                io.setPower(power);
            },
            () -> io.setPower(0.0)
        );
    }

    public Command stow() {
        return targetDegrees(MIN_DEGREES);
    }

    public Command targetDegrees(double degrees) {
        return run(() -> {
            setAngleTarget(degrees);
        });
    }

    public Command targetDegreesShotTable() {
        return run(
            () -> {
                AimOnTheMoveResult result = RobotContainer.instance.turret.getLastAOTMResult();
                if (result != null) {
                    setAngleTarget(result.hoodAngleDegrees, result.hoodVelocityRPS);
                }
            }
        );
    }

    public Command targetDegreesDistance(double distance) {
        return run(() -> {
            setAngleTarget(ShotTables.getHubShotTable().getHoodAngle(distance), 0);
        });
    }

    private void setAngleTarget(double degrees) {
        setAngleTarget(degrees, 0);
    }

    private void setAngleTarget(double degrees, double velocityRPS) {
        pidControl = true;
        targetPositionDegrees = Math.max(MIN_DEGREES, Math.min(MAX_DEGREES, degrees));
        targetVelocityRPS = velocityRPS;
    }
}
