package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ShooterIO {

    @Logged
    public class ShooterInputs {
        public double velocityRPS = 0;
        public double appliedVoltage = 0;
        public double supplyCurrent = 0;
        public double supplyCurrentFollower = 0;
        public double statorCurrent = 0;
        public double fuelDistance = 0;
        public boolean canrangeConnected = false;
    }

    public void setPower(double power);

    public void update(ShooterInputs inputs);

    public void setVelocityRPS(double velocity);

    public void setVelocityRPS(double velocity, double feedforwardVolts);

    public void setVelocityRPS(double velocity, double feedforwardVolts, int slot);

    public void simulationPeriodic();

}
