package frc.robot.subsystems.shooter;

public class ShooterIOFake implements ShooterIO {

    @Override
    public void setPower(double power) {}

    @Override
    public void update(ShooterInputs inputs) {}

    @Override
    public void setVelocityRPS(double velocity) {}

    @Override
    public void setVelocityRPS(double velocity, double feedforwardVolts) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void setVelocityRPS(double velocity, double feedforwardVolts, int slot) {}
}
