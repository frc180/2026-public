package frc.robot.subsystems.kicker;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.kicker.KickerIO.KickerInputs;

@Logged
public class KickerSubsystem extends SubsystemBase {

    static final double MAX_RPS = 95;
    public static final double SHOOT_RPS = 90;

    private final KickerIO io;
    private final KickerInputs inputs;

    double targetRPS = 0;
    boolean wasAtTarget = false;
    double timeSinceLostTarget = 0;

    public KickerSubsystem() {
        io = new KickerIOTalonFX();
        inputs = new KickerInputs();
    }

    @Override
    public void periodic() {
        io.update(inputs);
        
        boolean atTarget = isAtTarget();
        if (!atTarget && wasAtTarget) {
            timeSinceLostTarget = Timer.getFPGATimestamp();
        }
        wasAtTarget = atTarget;
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public double getSecondsSinceLostTarget() {
        return Timer.getFPGATimestamp() - timeSinceLostTarget;
    }

    public Command runForShoot() {
        return runPower(1);
    }

    public Command runPower(double power) {
        return runEnd(
            () -> io.setVoltage(power * 12),
            () -> io.setVoltage(0)
        );
    }

    public Command runRPS(double rps) {
        return runEnd(
            () -> ioSetVelocity(rps),
            () -> {
                targetRPS = 0;
                io.setVoltage(0);
            }
        );
    }

    public boolean isRunning () {
        return Math.abs(inputs.velocityRps) > 0.02;
    }

    public boolean isAtTarget() {
        return Math.abs(targetRPS - inputs.velocityRps) < 3.5;
    }

    private void ioSetVelocity(double rps) {
        targetRPS = rps;
        io.setVelocity(rps);
    }
}
