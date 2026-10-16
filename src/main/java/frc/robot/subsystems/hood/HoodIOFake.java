package frc.robot.subsystems.hood;

public class HoodIOFake implements HoodIO {

    @Override
    public void update(HoodInputs inputs) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void setAngle(double degrees) {}

    @Override
    public void setPower(double power) {}

    @Override
    public void setAngle(double degrees, double velocityRPS) {}

    @Override
    public void resetAngle(double degrees) {}

    @Override
    public void resetAngleNonBlocking(double degrees) {} 
}
