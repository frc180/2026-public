package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import com.spamrobotics.util.SubsystemExt;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterIO.ShooterInputs;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShotTables;
import frc.robot.util.AimOnTheMove.AimOnTheMoveResult;

@Logged
public class ShooterSubsystem extends SubsystemExt {

    private final ShooterIO io;
    private final ShooterInputs inputs;

    private final double ALLOWED_RPS_MARGIN = 1;

    private static final double FUEL_DETECT_THRESHOLD = 0.12;
    private static final double FUEL_UNDETECT_THRESHOLD = 0.12;

    double targetVelocityRPS = 0;
    boolean targetingLower = false;
    boolean voltageMode = true;

    double currentMargin = ALLOWED_RPS_MARGIN;

    int pidSlot = 0;

    int totalFuelCount = 0;
    int fuelCount = 0;
    boolean fuelDetected = false;
    boolean noFuelFlowing = true;
    double fuelVolleyStartTime = 0;
    double lastFuelExitTime = 0;
    double fuelAvgPerSecond = 0;
    double fuelRunningAvgPerSecond = 0;

    // Moving average over last 5 fuel balls for running rate
    private final LinearFilter fuelRateFilter = LinearFilter.movingAverage(5);

    public ShooterSubsystem() {
        io = new ShooterIOTalonFX();
        inputs = new ShooterInputs();

        SmartDashboard.putNumber("Shooter % Adjust", 1);
    }

    @Override
    public void periodic() {
        io.update(inputs);

        double currentTime = Timer.getFPGATimestamp();
        boolean hoodOutOfTheWay = RobotContainer.instance.hood.getDegrees() > -8;// || Robot.isSimulation();

        if (hoodOutOfTheWay && !fuelDetected && inputs.fuelDistance < FUEL_DETECT_THRESHOLD) {
            fuelDetected = true;
            // Check if this is a new volley (3+ seconds since last fuel exited)
            if (fuelCount == 0 || currentTime - lastFuelExitTime > 3) {
                fuelCount = 0;
                fuelVolleyStartTime = currentTime;
                fuelRateFilter.reset();
                fuelRunningAvgPerSecond = 0;
            }
        } else if (fuelDetected && inputs.fuelDistance > FUEL_UNDETECT_THRESHOLD) {
            fuelDetected = false;

            // Calculate running average from instantaneous rate (after first fuel)
            if (fuelCount > 0 && lastFuelExitTime > 0) {
                double timeSinceLast = currentTime - lastFuelExitTime;
                if (timeSinceLast > 0) {
                    double instantaneousRate = 1.0 / timeSinceLast;
                    fuelRunningAvgPerSecond = fuelRateFilter.calculate(instantaneousRate);
                }
            }

            fuelCount++;
            totalFuelCount++;
            lastFuelExitTime = currentTime;

            // Calculate total average after each fuel passes
            double volleyDuration = lastFuelExitTime - fuelVolleyStartTime;
            if (volleyDuration > 0) {
                fuelAvgPerSecond = fuelCount / volleyDuration;
            }
        }

        noFuelFlowing = currentTime - lastFuelExitTime > 0.5;
    }

    @Override
    public void periodicAfterScheduler() {
        if (targetingLower && atTargetVelocity()) {
            targetingLower = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    @NotLogged
    public boolean isCanrangeConnected() {
        return inputs.canrangeConnected;
    }

    @NotLogged
    public boolean isNoFuelFlowing() {
        return noFuelFlowing;
    }

    @NotLogged
    public double getVelocityRPS() {
        return inputs.velocityRPS;
    }

    public boolean atLaunchTargetVelocity() {
        return atTargetVelocity() && targetVelocityRPS > 20;
    }

    public boolean atTargetVelocity() {
        return Math.abs(targetVelocityRPS - inputs.velocityRPS) < currentMargin;
    }

    public Command withTargetMargin(DoubleSupplier marginSupplier) {
        return Commands.runEnd(
            () -> currentMargin = marginSupplier.getAsDouble(),
            () -> currentMargin = ALLOWED_RPS_MARGIN
        ).finallyDo(() -> currentMargin = ALLOWED_RPS_MARGIN);
    }

    public Command withTargetMargin(double margin) {
        return Commands.runEnd(
            () -> currentMargin = margin,
            () -> currentMargin = ALLOWED_RPS_MARGIN
        ).finallyDo(() -> currentMargin = ALLOWED_RPS_MARGIN);
    }

    public Command runPower(double power) {
        return runEnd(
            () -> io.setPower(power),
            () -> io.setPower(0)
        );
    }

    public Command runVelocity(double velocity) {
        return runEnd(
            () -> ioSetVelocityRPS(velocity),
            () -> {
                io.setPower(0);
                targetVelocityRPS = 0;
            }
        );
    }

    public Command runVelocityDistance(double distance) {
        return runEnd(
            () -> {
                ioSetVelocityRPS(ShotTables.getHubShotTable().getFlywheelRPS(distance));
            },
            () -> {
                io.setPower(0);
                targetVelocityRPS = 0;
            }
        );
    }

    public Command runVelocityShotTable() {
        return runEnd(
            () -> {
                TurretSubsystem turret = RobotContainer.instance.turret;
                AimOnTheMoveResult result = turret.getLastAOTMResult();
                if (result != null) {
                    double rps = result.flywheelRPS;
                    rps *= SmartDashboard.getNumber("Shooter % Adjust", 1);
                    ioSetVelocityRPS(rps);
                }
            },
            () -> {
                io.setPower(0);
                targetVelocityRPS = 0;
            }
        );
    }


    private void ioSetVelocityRPS(double velocity) {
        if (targetVelocityRPS - velocity > 2) {
            targetingLower = true;
        } else if (targetVelocityRPS > velocity) {
            targetingLower = false;
        }
        targetVelocityRPS = velocity;

        io.setVelocityRPS(velocity, 0);
    }
}
