package frc.robot.subsystems.intake_extension;

public class IntakeExtensionIOFake implements IntakeExtensionIO {

    @Override
    public void update(Inputs inputs) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void setPosition(double position) {}

    @Override
    public void resetPosition(double position) {}

    @Override
    public void stop() {}

    @Override
    public void setPower(double power) {}

    @Override
    public void coast() {}

    @Override
    public void setPosition(double position, double maxVel, double maxAccel) {}
}
