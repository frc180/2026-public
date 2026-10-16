package frc.robot.commands;

import com.spamrobotics.util.DoubleFunction;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleCommand extends Command {

    final CommandXboxController controller;
    final RumbleType rumbleType;

    DoubleFunction rumbleFunction = null;
    double rumbleStrength = 0;

    double startTime = 0;

    public RumbleCommand(CommandXboxController controller, RumbleType rumbleType, double rumbleStrength) {
        this.controller = controller;
        this.rumbleType = rumbleType;
        this.rumbleStrength = rumbleStrength;
    }

    public RumbleCommand(CommandXboxController controller, RumbleType rumbleType, DoubleFunction rumbleFunction) {
        this.controller = controller;
        this.rumbleType = rumbleType;
        this.rumbleFunction = rumbleFunction;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute() {
        if (rumbleFunction != null) {
            rumbleStrength = rumbleFunction.apply(Timer.getFPGATimestamp() - startTime);
        }
        controller.setRumble(rumbleType, rumbleStrength);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(rumbleType, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}