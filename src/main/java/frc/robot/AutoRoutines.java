package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.Set;
import java.util.function.BooleanSupplier;
import com.spamrobotics.util.Helpers;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.intake_extension.IntakeExtensionSubsystem.Position;

public class AutoRoutines {

    public static boolean autoShoot = false;

    private final AutoFactory factory;
    private final RobotContainer rc;
    private final ChassisSpeeds zeroSpeeds = new ChassisSpeeds();

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
        rc = RobotContainer.instance;

        factory.bind("prepare_shoot", prepareShoot());
        factory.bind("intake", rc.intake.runIntake().alongWith(intakeExtend()));
        factory.bind("intake_fast", rc.intake.runPower(1).alongWith(intakeExtend()));
        factory.bind("intake_stop", rc.intake.runPower(0));
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = factory.newRoutine("Do Nothing Auto");

        routine.active().onTrue(rc.drivetrain.runOnce(() -> rc.drivetrain.drive(new ChassisSpeeds())));
        return routine;
    }

    public AutoRoutine leftHighLow() {
        return highLow(false);
    }

    public AutoRoutine leftMidLow() {
        return midLow(false);
    }

    public AutoRoutine rightHighLow() {
        return highLow(true);
    }

    public AutoRoutine rightMidLow() {
        return midLow(true);
    }

    public AutoRoutine leftHighBumpLow() {
        return bumpHighLow(false);
    }

    public AutoRoutine rightHighBumpLow() {
        return bumpHighLow(true);
    }

    public AutoRoutine middleHub() {
        return twoMid("HubPath", null, false, false);
    }

    public AutoRoutine leftFollow() {
        return twoMid("Follow1", null, false, false);
    }

    private AutoRoutine highLow(boolean left) {
        return twoMid("HighV2_L", "LowPath_Loop2", false, left);
    }

    private AutoRoutine bumpHighLow(boolean left) {
        return twoMid("HighBump_L", "LowPath_Loop2_Bump", false, left);
    }

    private AutoRoutine midLow(boolean left) {
        return twoMid("MidV2_L", "LowPath_Loop2", false, left);
    }


    public AutoRoutine calibration(boolean visionEnabled) {
        final AutoRoutine routine = factory.newRoutine("Calibration " + (visionEnabled ? "Vision" : "No Vision"));
        final AutoTrajectory traj = routine.trajectory("Calibration");

        routine.active().onTrue(Helpers.fastSequence(
            traj.resetOdometry().alongWith(rc.vision.odometryUpdatesEnabled(visionEnabled)),
            followTraj(traj)
        ));

        return routine;
    }

    public AutoRoutine calibrationDrive2_5m(boolean visionEnabled) {
        final AutoRoutine routine = factory.newRoutine("CalibrationDrive2.5m " + (visionEnabled ? "Vision" : "No Vision"));
        final AutoTrajectory traj = routine.trajectory("DriveStraight_2_5_m_one");

        routine.active().onTrue(Helpers.fastSequence(
            traj.resetOdometry().alongWith(rc.vision.odometryUpdatesEnabled(visionEnabled)),
            followTraj(traj)
        ));

        return routine;
    }

    private AutoRoutine twoMid(String path1, String path2, boolean depotSotf, boolean mirror) {
        boolean followAuto = path1.contains("Follow");

        final AutoRoutine routine = factory.newRoutine("Two Mid: " + path1 + " -> " + path2);
        AutoTrajectory firstCycle = routine.trajectory(path1);
        AutoTrajectory secondCycle = path2 != null ? routine.trajectory(path2) : null;
        AutoTrajectory depotCycle = routine.trajectory(depotSotf ? "DepotPath_Slow" : "DepotPath");
        AutoTrajectory exitPath = routine.trajectory("ExitPath");

        if (mirror) {
            firstCycle = firstCycle.mirrorY();
            if (secondCycle != null) secondCycle = secondCycle.mirrorY();
            depotCycle = depotCycle.mirrorY();
            exitPath = exitPath.mirrorY();
        }

        final Pose2d firstCycleEnd = firstCycle.getFinalPose().get();
        final Pose2d secondCycleStart = secondCycle != null ? secondCycle.getInitialPose().get() : null;
        final Pose2d secondCycleEnd = secondCycle != null ? secondCycle.getFinalPose().get() : null;

        Command initBonus = Commands.none();
        if (followAuto) {
            initBonus = Commands.defer(() -> {
                double arbWait = SmartDashboard.getNumber("FollowWait", 4.5);
                return Commands.waitSeconds(arbWait);
            }, Set.of());
        }

        Command init = Commands.parallel(
            visionOn(),
            firstCycle.resetOdometry(),
            rc.intakeExtension.targetPosition(Position.STOWED).withTimeout(0).asProxy(),
            initBonus
        );

        routine.active().onTrue(Helpers.fastSequence(
            init,
            followTraj(firstCycle)
        ));

        BooleanSupplier atFirstShootingPose = atPose(firstCycleEnd);

        Command shootAction;
        if (path1.contains("Bump")) {
            BooleanSupplier atSecondCycleStart = atPose(secondCycleStart);
            shootAction = new DriveToPose(rc.drivetrain, secondCycleStart)
                                .withMaxSpeed(0.1)
                                .withMaxAngularRate(DrivetrainSubsystem.MAX_ANGULAR_RATE * 0.1)
                                .alongWith(shootMoving(), jostleWithDelay(1))
                                .until(atSecondCycleStart)
                                .andThen(Commands.waitUntil(() -> rc.shooter.isNoFuelFlowing()).withTimeout(3));
        } else {
            shootAction = waitAndShoot(true, 1.5);
        }

        firstCycle.done().onTrue(Helpers.fastSequence(
            Commands.either(
                Commands.none(),
                followAuto ? Commands.none() : finishPath(firstCycleEnd, atFirstShootingPose),
                atFirstShootingPose
            ),

            shootAction,


            stopPrepareShoot(),
            Commands.waitUntil(rc.hood::atStowed),
            secondCycle != null ? followTraj(secondCycle) : Commands.none()
        ));

        if (secondCycle == null) {
            if (followAuto) {
                firstCycle.done().onTrue(Commands.waitSeconds(2).andThen(rc.intake.runPower(0)));
            }
            return routine;
        }

        if (depotSotf) {
            secondCycle.atTimeBeforeEnd(0.5).onTrue(
                followTraj(depotCycle).alongWith(Commands.waitSeconds(0.5).andThen(shootMoving()))
            );
        } else {
            BooleanSupplier atSecondShootingPose = atPose(secondCycleEnd);

            secondCycle.done().onTrue(Helpers.fastSequence(

                Commands.either(
                    Commands.none(),
                    finishPath(secondCycleEnd, atSecondShootingPose),
                    atSecondShootingPose
                ),


                waitAndShoot(false, 2),
                stopPrepareShoot(),
                Commands.waitUntil(rc.hood::atStowed),
                followAuto ? Commands.none() : followTraj(exitPath)
            ));
        }

        return routine;
    }

    private Command prepareShoot() {
        return Commands.parallel(
            rc.turretAim(),
            rc.shooter.runVelocityShotTable(),
            rc.flashlight.enable()
        );
    }

    private Command stopPrepareShoot() {
        return Commands.parallel(
            rc.shooter.runPower(0),
            disableShoot(),
            rc.hood.stow(),
            rc.flashlight.disable()
        ).withTimeout(0).asProxy();
    }

    private Command shootMoving() {
        return enableShoot().alongWith(
            rc.hood.targetDegreesShotTable().asProxy()
        );
    }

    private Command jostleWithDelay(double delay) {
        if (delay < 0) return Commands.none();

        Command jostle = rc.limitedIntakeJostle().withTimeout(4).asProxy();
        if (delay <= 0) return jostle;
        return Commands.waitSeconds(delay).andThen(jostle);
    }

    private Command waitAndShoot(boolean kickerStop, double jostleDelay) {
        Command jostle = jostleWithDelay(jostleDelay);

        Command shoot = Helpers.fastSequence(
            enableShoot().alongWith(kickerStop ? waitForShootingToComplete(5) : waitForShootingToComplete(20)),
            disableShoot()
        ).deadlineFor(rc.hood.targetDegreesShotTable().asProxy(), jostle);

        return Helpers.fastSequence(
            Commands.waitUntil(rc.drivetrain.belowSpeed(0.1)),
            shoot
        );
    }

    private Command intakeExtend() {
        return rc.intakeExtension.targetPosition(Position.EXTENDED);
    }

    private boolean dynamicIsNoFuelFlowing() {
        if (rc.shooter.isCanrangeConnected()) {
            return rc.shooter.isNoFuelFlowing();
        } else {
            return rc.noFuelFlowing.getAsBoolean();
        }
    }

    private Command waitForShootingToComplete(double maxTime) {
        return Helpers.fastSequence(
            Commands.waitSeconds(2),
            Commands.waitUntil(this::dynamicIsNoFuelFlowing).withTimeout(maxTime - 2) 
        );
    }

    private Command enableShoot() {
        return Commands.runOnce(() -> autoShoot = true);
    }

    private Command disableShoot() {
        return Commands.runOnce(() -> autoShoot = false);
    }

    private Command visionOn() {
        return rc.vision.odometryUpdatesEnabled(true);
    }

    /**
     * Follows a trajectory and then stops the drivetrain at the end.
     */
    private Command followTraj(AutoTrajectory traj) {
        return traj.cmd().finallyDo(() -> rc.drivetrain.drive(zeroSpeeds));
    }

    private Command finishPath(Pose2d end, BooleanSupplier atEnd) {
        return Helpers.fastSequence(
            goTrajEnd(end, atEnd).withTimeout(2),
            Commands.either(
                Commands.none(),
                Commands.defer(() -> {
                    Pose2d retry = new Pose2d(
                        Robot.isBlue() ? 6.5 : Robot.FIELD_LENGTH - 6.5,
                        end.getY(),
                        end.getRotation()
                    );
                    return new DriveToPose(rc.drivetrain, retry)
                            .withMaxSpeed(0.75)
                            .until(atPose(retry))
                            .andThen(goTrajEnd(end, atEnd));
                }, Set.of(rc.drivetrain)),
                atEnd
            )
        );
    }

    private Command goTrajEnd(Pose2d end, BooleanSupplier atEnd) {
        return new DriveToPose(rc.drivetrain, end)
                            .withMaxSpeed(0.75)
                            .until(atEnd);

    }

    private BooleanSupplier atPose(Pose2d pose) {
        return () -> withinPoseTolerance(
            pose,
            Meters.of(0.1),
            Meters.of(0.1),
            Degrees.of(8)
        );
    }

    private boolean withinPoseTolerance(Pose2d targetPose, Distance xDistance, Distance yDistance, Angle angle) {
        return Helpers.withinTolerance(rc.drivetrain.getPose(), targetPose, xDistance, yDistance, angle);
    }
}