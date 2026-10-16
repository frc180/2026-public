package frc.robot.subsystems.turret;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.epilogue.NotLogged;

public class TurretIOFake implements TurretIO {

    @Override
    public void update(TurretInputs inputs) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void setSpeed(double power) {}

    @Override
    public void setPositionTarget(double degrees) {}

    @Override
    public void setPositionTarget(double degrees, double feedForwardVolts) {}

    @Override
    public void setPositionTarget(double degrees, double rotPerSec, int slot) {}

    @Override
    public void resetPosition(double absoluteDegrees) {}

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return new ArrayList<>();
    }

    @Override
    public void fastUpdate(FastTurretInputs inputs) {}

    @Override
    public void coast() {}

    @NotLogged
    @Override
    public boolean enableSoftwareLimits() {
        return true;
    }
}
