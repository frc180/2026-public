package frc.robot.subsystems.kicker;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface KickerIO {

    @Logged
    public class KickerInputs {
        public double appliedVolts = 0;
        public double velocityRps = 0;
        public double supplyCurrent = 0;
        public double statorCurrent = 0;
    }

    public void update(KickerInputs inputs);

    public void simulationPeriodic();

    public void setVoltage(double volts);

    public void setVelocity(double rps);

    public void setVelocityTorqueCurrent(double rps);

    default public void stop() {
        setVoltage(0);
    }
}
