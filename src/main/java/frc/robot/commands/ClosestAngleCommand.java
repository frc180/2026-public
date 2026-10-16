package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
 
public class ClosestAngleCommand extends Command {

    private final DrivetrainSubsystem drivetrain;

    private final double[] angles;

    private double targetDegrees = 0;


    public ClosestAngleCommand(DrivetrainSubsystem drivetrain, double... angles) {
        this.drivetrain = drivetrain;
        this.angles = angles;
    }


    @Override
    public void initialize () {
        double currentDegrees = drivetrain.getGyroscopeDegrees();
        targetDegrees = findClosestAngle(currentDegrees);
        drivetrain.setTargetHeading(targetDegrees);
    }

    @Override
    public void execute() {
        drivetrain.setTargetHeading(targetDegrees);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

 static double normalizeAngle(double angleDegrees) {
        double angle = angleDegrees % 360;
        angle = (angle + 360) % 360;
        if (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    private double findClosestAngle(double currentDeg) {
        double best = angles[0];
        double minError = Double.MAX_VALUE;
        for (double d : angles) {
            double error = Math.abs(
                normalizeAngle(d - currentDeg)
            );
            if (error < minError) {
                minError = error;
                best = d;
            }
        }
        return best;
    }
}