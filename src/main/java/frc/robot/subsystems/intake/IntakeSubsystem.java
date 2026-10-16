package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;
import edu.wpi.first.epilogue.Logged;

@Logged
public class IntakeSubsystem extends SubsystemBase {

    private static final double RPS_12V = 100;

    private static final double INTAKE_RPS = 65;
    private static final double SPIT_RPS = -INTAKE_RPS;

    private final IntakeIO io;
    private final IntakeInputs inputs;

    private double targetVelocityRps = 0;
    

    public IntakeSubsystem() {
        io = new IntakeIOTalonFX();
        inputs = new IntakeInputs();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command runIntake() {
        return runPower(1);
    }

    public Command runSpit() {
        return runVelocity(SPIT_RPS);
    }

    public Command runJostle() {
        return runPower(1);
    }

    public Command runPower(double power) {
        return runEnd(
            () -> {
                io.setVoltage(power * 12);
                targetVelocityRps = power * RPS_12V;
            },
            () -> {
                io.setVoltage(0);
                targetVelocityRps = 0;
            }
        );
    }

    public Command runVelocity(double rps) {
        return runEnd(
            () -> {
                io.setVelocity(rps);
                targetVelocityRps = rps;
            },
            () -> {
                io.setVoltage(0);
                targetVelocityRps = 0;
            }
        );
    }
}
