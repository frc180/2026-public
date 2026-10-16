package frc.robot.subsystems.kicker;

public class KickerIOFake implements KickerIO {

    @Override
    public void update(KickerInputs inputs) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void setVoltage(double volts) {}

    @Override
    public void setVelocity(double rps) {}

    @Override
    public void setVelocityTorqueCurrent(double rps) {}
}
