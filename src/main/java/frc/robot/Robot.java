// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import com.spamrobotics.util.SubsystemExt;
import frc.robot.util.ShotTables;
import frc.robot.util.HubStatus;
import frc.robot.util.StatusSignals;
import frc.robot.util.simulation.SimUtils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.VisionSubsystem;


@Logged(importance = Logged.Importance.CRITICAL)
public class Robot extends TimedRobot {

    public static final AprilTagFieldLayout aprilTagFieldLayout = VisionSubsystem.loadAprilTagFieldLayout();
    public static final double FIELD_LENGTH = aprilTagFieldLayout.getFieldLength();
    public static final double FIELD_WIDTH = aprilTagFieldLayout.getFieldWidth();

    private static Robot self = null;


    private Command m_autonomousCommand;
    private String autoName = "null";
    private boolean rightAuto = false;

    private final RobotContainer m_robotContainer;

    private final Alert controllerDisconnectedAlert = new Alert("Setup - Driver controller not connected!", AlertType.kError);

    private boolean isBlueAlliance = true;
    private boolean isEnabledCached = false;
    private boolean isAutonomousCached = false;
    private double distanceOffsetCached = 0;

    private double teleopStartTime = 0;
    private double timeRemainingSeconds = 999;
    @NotLogged
    private Alliance allianceWonAuto = null;
    private boolean hubActive = true;
    private double timeUntilNextShift = -1;
    private double timeUntilNextActiveShift = -1;
    private double timeUntilNextInactiveShift = -1;
    private double lastShiftActiveTime = 0;

    private String displayTimeUntilActive = "0";
    private String displayTimeUntilInactive = "0";

    private double voltage = 0;

    public Robot() {
        if (Robot.isSimulation()) {
            Arena2026Rebuilt test = new Arena2026Rebuilt(false);
            test.setEfficiencyMode(true);
            SimulatedArena.overrideInstance(test);
        }

        ShotTables.init();
        self = this;
        m_robotContainer = new RobotContainer();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Epilogue.getConfig().minimumImportance = Importance.DEBUG; // INFO for comp matches, DEBUG otherwise
        Epilogue.bind(this);

        SmartDashboard.putNumber("DistanceOffset", 0);
    }

    @Override
    public void robotPeriodic() {
        isEnabledCached = RobotState.isEnabled();
        isAutonomousCached = RobotState.isAutonomous();
        distanceOffsetCached = SmartDashboard.getNumber("DistanceOffset", 0);

        if (allianceWonAuto == null) {
            String gamedata = DriverStation.getGameSpecificMessage();
            if (gamedata.length() >= 1) {
                allianceWonAuto = (gamedata.charAt(0) == 'B') ? Alliance.Blue : Alliance.Red;
            }
        }

        isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        if (teleopStartTime > 0) {
            double elapsed = (Timer.getFPGATimestamp() - teleopStartTime);
            timeRemainingSeconds = 140 - elapsed;
        } else { 
            timeRemainingSeconds = 140;
        }

        m_robotContainer.receivedAutoWinner = allianceWonAuto != null;
        if (m_robotContainer.hasAppliedAutoWinnerOverride) {
            allianceWonAuto = m_robotContainer.autoWinnerBlue ? Alliance.Blue : Alliance.Red;
        }

        if (allianceWonAuto != null) {
            double remainingSeconds = timeRemainingSeconds;
            hubActive = HubStatus.allianceActive(
                remainingSeconds,
                self.isBlueAlliance,
                self.allianceWonAuto == Alliance.Blue
            );
            timeUntilNextShift = HubStatus.secondsUntilNextShift(remainingSeconds);
            timeUntilNextActiveShift = HubStatus.secondsUntilNextActiveShift(
                remainingSeconds,
                self.isBlueAlliance,
                self.allianceWonAuto == Alliance.Blue
            );
            timeUntilNextInactiveShift = HubStatus.secondsUntilNextInactiveShift(
                remainingSeconds,
                self.isBlueAlliance,
                self.allianceWonAuto == Alliance.Blue
            );
            
            if (timeUntilNextActiveShift > -1) {
                displayTimeUntilActive = String.format("%.1f", Math.max(0, timeUntilNextActiveShift));
            } else {
                displayTimeUntilActive = "-";
            }
            if (timeUntilNextInactiveShift > -1) {
                displayTimeUntilInactive = String.format("%.1f", Math.max(0, timeUntilNextInactiveShift));
            } else {
                displayTimeUntilInactive = "-";
            }
            
            if (timeUntilNextActiveShift < 0 && timeUntilNextInactiveShift < 0) {
                displayTimeUntilInactive = String.format("%.1f", Math.max(0, remainingSeconds));
            }
            
        } else {
            hubActive = true;
        }
        if (hubActive) {
            lastShiftActiveTime = Timer.getFPGATimestamp();
        }

        m_robotContainer.drivetrain.clearCache();
        StatusSignals.refreshAll();
        CommandScheduler.getInstance().run();
        SubsystemExt.ALL_SUBSYSTEMS.forEach(SubsystemExt::periodicAfterScheduler);
        StatusSignals.updateDisconnectedAlerts();

        voltage = RobotController.getBatteryVoltage();


        Pose3d[] mechanismPoses = { m_robotContainer.turret.getVisPose(), m_robotContainer.intakeExtension.getVisPose() };
        mechanismPosePublisher.accept(mechanismPoses);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        controllerDisconnectedAlert.set(!m_robotContainer.driverController.isConnected());
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        AutoRoutines.autoShoot = false;
        CommandScheduler.getInstance().cancelAll();
        if (Robot.isSimulation()) {
            SimulatedArena.getInstance().resetFieldForAuto();
            SimUtils.intakeSimulation.setGamePiecesCount(0);
            SimUtils.setHopperFuel(8);
        }
        allianceWonAuto = null;
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            autoName = m_autonomousCommand.getName();
            char autoSide = autoName.charAt(0);
            rightAuto = autoSide == 'R';
            boolean middleAuto = autoSide == 'M';

            if (middleAuto) {
                m_robotContainer.drivetrain.setGyroscopeAngle(180);
            } else {
                double angle = rightAuto ? 90 : -90;
                m_robotContainer.drivetrain.setGyroscopeAngle(angle);
            }
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        AutoRoutines.autoShoot = false;
        m_robotContainer.vision.setOdometryUpdatesEnabled(true);
        teleopStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    public static boolean isBlue() {
        return self.isBlueAlliance;
    }

    public static boolean isRed() {
        return !isBlue();
    }

    public static boolean isEnabledCached() {
        return self.isEnabledCached;
    }

    public static boolean isDisabledCached() {
        return !isEnabledCached();
    }

    public static boolean isAutonomousCached() {
        return self.isAutonomousCached;
    }

    public static double getDistanceOffset() {
        return self.distanceOffsetCached;
    }

    public static boolean hubActive() {
        return self.hubActive;
    }

    public static double timeUntilNextShift() {
        return self.timeUntilNextShift;
    }

    public static double timeUntilNextActiveShift() {
        return self.timeUntilNextActiveShift;
    }

    public static double timeUntilNextInactiveShift() {
        return self.timeUntilNextInactiveShift;
    }

    public static double lastShiftActiveTime() {
        return self.lastShiftActiveTime;
    }

    // ============================= SIMULATION ONLY =============================

    final StructArrayPublisher<Pose3d> fuelPosePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Fuel Poses", Pose3d.struct)
            .publish();

    final StructArrayPublisher<Pose3d> mechanismPosePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Mechanism Poses", Pose3d.struct)
            .publish();

    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public void simulationPeriodic() {
        SimUtils.periodic();
        SimulatedArena arena = SimulatedArena.getInstance();
        Pose3d[] fieldFuelPoses = arena.getGamePiecesArrayByType("Fuel");

        // Calculate poses for fuel stored inside the robot for visualization
        int storedFuelCount = SimUtils.getFuelCount();
        Pose3d[] allFuelPoses = new Pose3d[fieldFuelPoses.length + storedFuelCount];
        
        // Copy field fuel to the combined array
        System.arraycopy(fieldFuelPoses, 0, allFuelPoses, 0, fieldFuelPoses.length);

        // // Generate poses for stored fuel relative to the robot
        Pose2d robotPose2d = m_robotContainer.drivetrain.getDriveSim().getSimulatedDriveTrainPose();
        Pose3d robotPose3d = new Pose3d(robotPose2d);
        simFuelStack(allFuelPoses, fieldFuelPoses, robotPose3d);

        fuelPosePublisher.accept(allFuelPoses);
    }

    private void simFuelStack(Pose3d[] allFuelPoses, Pose3d[] fieldFuelPoses, Pose3d robotPose3d) {
        final int GRID_DEPTH_X = 4; // How many fuel deep (X-axis) before stacking up
        final int GRID_WIDTH_Y = 6; // How many fuel wide (Y-axis) before stacking up

        int storedFuelCount = SimUtils.getFuelCount();

        for (int i = 0; i < storedFuelCount; i++) {
            int itemsPerLayer = GRID_DEPTH_X * GRID_WIDTH_Y;
            
            // Calculate grid indices
            int zLevel = i / itemsPerLayer;         // Vertical Stack Index
            int indexInLayer = i % itemsPerLayer;   // Index within the flat layer
            int row = indexInLayer / GRID_WIDTH_Y;  // X-axis index
            int col = indexInLayer % GRID_WIDTH_Y;  // Y-axis index

            // Offsets in meters relative to robot center (X is forward, Y is left, Z is up)
            double xOffset = 0.1 + (row * 0.1);    // Start slightly behind center and stack backwards
            double yOffset = -0.25 + (col * 0.1);     // Spread width-wise
            double zOffset = 0.25 + (zLevel * 0.1); // Base height + stack height (adjust for fuel diameter)

            Transform3d storageTransform = new Transform3d(
                new Translation3d(xOffset, yOffset, zOffset), 
                new Rotation3d()
            );

            allFuelPoses[fieldFuelPoses.length + i] = robotPose3d.transformBy(storageTransform);
        }
    }
}
