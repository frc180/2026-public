package frc.robot.subsystems.prekicker;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.prekicker.PrekickerIO.PrekickerInputs;

@Logged
public class Prekicker extends SubsystemBase {

    static final double RPS_12V = 67;

    public static final double DEFAULT_POW = 0.5;

    final PrekickerIO io;
    final PrekickerInputs inputs;

    double targetRPS = 0;

    public Prekicker() {
        io = new PrekickerIOTalonFX();
        inputs = new PrekickerInputs();
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }
    
    public Command runPower(double power) {
        return runEnd(
            () -> {
                io.setPower(power);
                targetRPS = RPS_12V * power;
            },
            () -> {
                io.setPower(0);
                targetRPS = 0;
            }
        );
    }

    public Command runVelocity(double rps) {
        return runEnd(
            () -> {
                io.setVelocity(rps);
                targetRPS = rps;
            },
            () -> {
                io.setPower(0);
                targetRPS = 0;
            }
        );  
    }
}
