package frc.robot.subsystems.intake_extension;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeExtensionIO {

    @Logged
    public class Inputs {
        double position = 0.0;
        double targetPosition = 0;
        double supplyCurrent = 0;
    }

    void update(Inputs inputs);

    void simulationPeriodic();

    void coast();

    void setPower(double power);

    void setPosition(double position);

    void setPosition(double position, double maxVel, double maxAccel);

    void resetPosition(double position);

    void stop();
}
