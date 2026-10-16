package frc.robot.subsystems.hood;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface HoodIO {

    @Logged
    public class HoodInputs {
        double positionDegrees = 0.0;
        double targetDegrees = 0;
        double supplyCurrent = 0;
    }

    void update(HoodInputs inputs);

    void simulationPeriodic();

    void setPower(double power);

    void setAngle(double degrees);

    void setAngle(double degrees, double velocityRPS);

    void resetAngle(double degrees);

    void resetAngleNonBlocking(double degrees);
}
