package frc.robot.subsystems.turret;

import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;

@Logged
public interface TurretIO {

    @Logged
    public class FastTurretInputs {
        double positionDegrees = 0;
        double velocityRPS = 0;
    }

    @Logged
    public class TurretInputs {
        double absolutePositionDegrees = 0;
        double absolutePositionRot = 0;
        double secondaryAbsolutePositionDegrees = 0;
        double secondaryAbsolutePositionRot = 0;
        double targetDegrees = 0;
        double supplyCurrent = 0;
    }

    public List<BaseStatusSignal> getStatusSignals();

    public void fastUpdate(FastTurretInputs inputs);

    public void update(TurretInputs inputs);

    public void simulationPeriodic();

    public void setSpeed(double power);

    public void setPositionTarget(double degrees);

    public void setPositionTarget(double degrees, double rotPerSec);

    public void setPositionTarget(double degrees, double rotPerSec, int slot);

    public void resetPosition(double absoluteDegrees);

    @NotLogged
    public boolean enableSoftwareLimits();

    public void coast();
}
