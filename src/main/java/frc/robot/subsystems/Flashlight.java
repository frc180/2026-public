package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Flashlight extends SubsystemBase {
    
    final Relay flashlight;
    final boolean hardwareEnabled;
    final PowerDistribution pdh;

    public Flashlight(boolean enabled, PowerDistribution powerDistribution) {
        hardwareEnabled = enabled;
        if (hardwareEnabled && powerDistribution == null) {
            flashlight = new Relay(0, Direction.kForward);
        } else { 
            flashlight = null;
        }

        pdh = powerDistribution;
    }

    public Command enabledWhenAppropriate() {
        TurretSubsystem turret = RobotContainer.instance.turret;
        return enabled(() -> {
            return !turret.isWrapping() && turret.isTargetingHub() && turret.getTargetError() < 15;
        }).finallyDo(() -> setEnabled(false));
    }

    public Command runEnabled() {
        return run(() -> setEnabled(true))
                .finallyDo(() -> setEnabled(false));
    }

    public Command enabled(BooleanSupplier enableSupplier) {
        return run(() -> setEnabled(enableSupplier.getAsBoolean()));
    }

    public Command enable() {
        return runOnce(() -> setEnabled(true))
                .ignoringDisable(true);
    }

    public Command disable()    {
        return runOnce(() -> setEnabled(false))
                .ignoringDisable(true);
    }

    private void setEnabled(boolean enabled) {
        if (!hardwareEnabled) return;

        if (pdh != null) {
            pdh.setSwitchableChannel(enabled);
        } else {
            flashlight.set(enabled ? Value.kForward : Value.kOff);
        }
    }
}
