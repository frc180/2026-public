package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.IndexerIO.IndexerInputs;

@Logged
public class IndexerSubsystem extends SubsystemBase {

    private static final double MAX_RPS = 11.2;
    private static final double JOSTLE_THRESHOLD = 0.18;

    private final IndexerIO io;
    private final IndexerInputs inputs;

    double targetRPS = 0;
    boolean currentlyStalling = false;

    final Trigger stalling = new Trigger(() -> currentlyStalling).debounce(0.75, DebounceType.kRising);
    final Alert stallingAlert = new Alert("Indexer stalling", AlertType.kWarning);

    public IndexerSubsystem() {
        io = new IndexerIOTalonFX();
        inputs = new IndexerInputs();
    }

    @Override
    public void periodic() {
        io.update(inputs);

        currentlyStalling = targetRPS > 0 && inputs.velocityRPS < 2;
    
        stallingAlert.set(stalling.getAsBoolean());
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command runForShoot() {
        return runPower(1);
    }

    public Command runPower(double power) {
        return runEnd(
            () -> ioSetPower(power),
            () -> {
                ioSetPower(0);
            }
        );
    }

    public Command runVelocity(double rps) {
        return runEnd(
            () -> ioSetVelocity(rps),
            () -> {
                ioSetPower(0);
            }
        );  
    }

    public Command runVariableVelocityPercent(DoubleSupplier percent) {
        return runEnd(
            () -> ioSetVelocity(MAX_RPS * percent.getAsDouble()),
            () -> ioSetPower(0)
        );
    }

    public Command runVelocityPercent(double percent) {
        return runVelocity(MAX_RPS * percent);
    }

    public Command goToPosition(double degrees) {
        return run( 
            () -> io.setPositionDegrees(degrees));
    }

    public boolean isRunning() {
        return Math.abs(inputs.velocityRPS) > 0.02;
    }

    public boolean canJostle() {
        return inputs.hopperDistance > JOSTLE_THRESHOLD;
    }

    private void ioSetVelocity(double rps) {
        targetRPS = rps;
        io.setVelocity(rps);
    }

    private void ioSetPower(double power) {
        targetRPS = power * MAX_RPS;
        io.setPower(power);
    }
}
