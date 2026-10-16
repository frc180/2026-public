package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.hood.HoodSubystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public abstract class SimUtils {

    public static IntakeSimulation intakeSimulation = null;
    public static boolean justLaunchedFuel = false;
    private static int hopperFuel = 0;
    private static List<Double> processingFuels = new ArrayList<>();
    
    public static final int MAX_FUEL = 48;
    public static final double FUEL_PER_SECOND = 8;
    public static final double INTAKE_TO_HOPPER_SECONDS = 0.3;
    public static final Distance HUB_HEIGHT = Inches.of(72);
    public static final Distance SHOOTER_HEIGHT = Inches.of(20.878738);
    public static final Distance FLYWHEEL_RADIUS = Inches.of(2);

    public static int getFuelCount() {
        if (intakeSimulation == null) {
            return 0;
        }
        return hopperFuel;
    }

    public static int getCombinedFuelCount() {
        return hopperFuel + intakeSimulation.getGamePiecesAmount();
    }

    public static void setHopperFuel(int count) {
        hopperFuel = count;
    }

    public static boolean obtainFuelFromHopper() {
        if (hopperFuel > 0) {
            hopperFuel--;
            return true;
        }
        return false;
    }

    public static void periodic() {
        double timestamp = Timer.getFPGATimestamp();

        for (Double fuelTimestamp : new ArrayList<>(processingFuels)) {
            if (timestamp - fuelTimestamp > INTAKE_TO_HOPPER_SECONDS) {
                hopperFuel++;
                processingFuels.remove(fuelTimestamp);
                if (hopperFuel > MAX_FUEL) hopperFuel = MAX_FUEL;
            }
        }

        while (intakeSimulation.obtainGamePieceFromIntake()) {
            processingFuels.add(timestamp);
        }
    }

    public static void shootFuel() {
        DrivetrainSubsystem drive = RobotContainer.instance.drivetrain;
        TurretSubsystem turret = RobotContainer.instance.turret;
        ShooterSubsystem shooter = RobotContainer.instance.shooter;
        IndexerSubsystem indexer = RobotContainer.instance.indexer;
        KickerSubsystem kicker = RobotContainer.instance.kicker;
        HoodSubystem hood = RobotContainer.instance.hood;

        // Indexer and Kicker requirements for shooting fuel
         if (!indexer.isRunning() || !kicker.isRunning() || intakeSimulation == null || !obtainFuelFromHopper()) {
            return;
        }

        Pose2d turretPose = turret.getSimPose();
        Rotation2d shooterAngle = turretPose.getRotation();
        LinearVelocity shooterVelocity = MetersPerSecond.of(calculateExitVelocity(shooter.getVelocityRPS()));
        Angle shooterLaunchAngle = Degrees.of(hood.getDegrees());

        RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
            turretPose.getTranslation(),
            Translation2d.kZero,
            drive.getFieldRelativeSpeeds(),
            shooterAngle,
            SHOOTER_HEIGHT,
            shooterVelocity,
            shooterLaunchAngle
        );
        SimulatedArena.getInstance().addGamePieceProjectile(fuel);
        justLaunchedFuel = true;
    }

    public static double calculateExitVelocity(double shooterRPS) {        
        final double HOODED_MECHANICAL_FACTOR = 0.5; 
        final double FLYWHEEL_EFFICIENCY = 0.9; 

        double angularVelocityRadPerSec = shooterRPS * 2 * Math.PI; // RPS to radians per second
        double surfaceVelocity = FLYWHEEL_RADIUS.in(Meters) * angularVelocityRadPerSec; // in meters per second

        return surfaceVelocity * HOODED_MECHANICAL_FACTOR * FLYWHEEL_EFFICIENCY;
    }

    /**
     * 
     * @param v Initial launch velocity in m/s
     * @param angleDeg Launch angle in degrees
     * @param hStart Height at which the ball is launched
     * @param hEnd Height at which the ball is supposed to go through
     * @return Time of flight in seconds, or -1 if the ball cannot reach the target height
     */
    public static double calculateIdealTimeOfFlight(double v, double angleDeg, double hStart, double hEnd) {
        double g = 9.81;
        double angleRad = Math.toRadians(angleDeg);
        double vy = v * Math.sin(angleRad);

        double a = -0.5 * g;
        double b = vy;
        double c = hStart - hEnd;

        double discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0) {
            return -1; // Ball didn't have enough height/speed to reach the rim
        }

        double sqrtDisc = Math.sqrt(discriminant);
        
        double t1 = (-b + sqrtDisc) / (2 * a);
        double t2 = (-b - sqrtDisc) / (2 * a);

        return Math.max(t1, t2);
    }   
}
