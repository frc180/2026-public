package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeIO {

    @Logged
    public class IntakeInputs {
        public double velocityRps = 0;
        public double supplyCurrent = 0;
        public double secondarySupplyCurrent = 0;
    }

    public void update(IntakeInputs inputs);

    public void simulationPeriodic();

    public void setVoltage(double volts);

    public void setVelocity(double rps);

    default public void stop() {
        setVoltage(0);
    }
}
