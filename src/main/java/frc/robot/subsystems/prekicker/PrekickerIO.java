package frc.robot.subsystems.prekicker;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface PrekickerIO {

    @Logged
    public class PrekickerInputs {
        double velocityRPS = 0;
    }

    public void update(PrekickerInputs inputs);

    public void setPower(double power);

    public void setVelocity(double rps);
    
}
