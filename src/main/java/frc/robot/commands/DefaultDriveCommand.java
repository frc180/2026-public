package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.spamrobotics.util.JoystickInputs;
import com.spamrobotics.util.RobotCentricExt;

public class DefaultDriveCommand extends Command {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final RobotCentricExt robotCentricControl;


    private final Supplier<JoystickInputs> m_joystickInputsSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private BooleanSupplier slowModeSupplier;

    private final BooleanSupplier m_shootButtonHeld;


    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;


    private boolean manuallyRotating;
    private Rotation2d gyroRotation;
    private HeadingTarget previousHeadingType = null;
    private double rotationSpeed = 0;
    private boolean slowMode = false;
    private boolean brakeEngaged = false;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
     
     
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                Supplier<JoystickInputs> joystickInputsSupplier,
                                DoubleSupplier rotationSupplier, BooleanSupplier shootButtonHeld) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_joystickInputsSupplier = joystickInputsSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_shootButtonHeld = shootButtonHeld;


        robotCentricControl = (RobotCentricExt) new RobotCentricExt().withDesaturateWheelSpeeds(false);

        xLimiter = new SlewRateLimiter(10);
        yLimiter = new SlewRateLimiter(10);

        addRequirements(drivetrainSubsystem);
    }
     
    public DefaultDriveCommand withSlowMode(BooleanSupplier slowModeSupplier) {
        this.slowModeSupplier = slowModeSupplier;
        return this;
    }

    @Override
    public void initialize() {
        manuallyRotating = false;
        m_drivetrainSubsystem.setTargetHeading(null);
    }

    @Override
    public void execute() {
        gyroRotation = m_drivetrainSubsystem.getGyroscopeRotation();
        rotationSpeed = m_rotationSupplier.getAsDouble();
    
        if (Math.abs(rotationSpeed) < 0.02) {
            // If we were manually rotating and have stopped, save this heading as our new target
            if (manuallyRotating) {
                m_drivetrainSubsystem.setTargetHeading(null);
                previousHeadingType = null;
            }

            Double targetHeadingDegrees = m_drivetrainSubsystem.getTargetHeading();
            if (targetHeadingDegrees != null) {
                HeadingTarget headingType = m_drivetrainSubsystem.getTargetHeadingType();
                Rotation2d heading = headingType == HeadingTarget.POSE ? m_drivetrainSubsystem.getPose().getRotation() : gyroRotation;
                if (headingType != previousHeadingType) {
                    // Reset the PID controller if the heading type has changed
                    m_drivetrainSubsystem.resetHeadingPID(heading);
                }
                rotationSpeed = m_drivetrainSubsystem.calculateHeadingPID(heading, targetHeadingDegrees);
                previousHeadingType = headingType;
            }
            manuallyRotating = false;
        } else {
            manuallyRotating = true;
        }

        JoystickInputs inputs = m_joystickInputsSupplier.get();

        boolean requestingTranslation = Math.abs(inputs.x) > 0.02 || Math.abs(inputs.y) > 0.02;
        boolean stopped = m_drivetrainSubsystem.isBelowSpeed(0.01);
        boolean shooting = m_shootButtonHeld.getAsBoolean();
        boolean rotating = Math.abs(rotationSpeed) > 0.02;

        if (!requestingTranslation && stopped && shooting && !rotating) {
            brakeEngaged = true;
        }
        if (requestingTranslation || !shooting || rotating) {
            brakeEngaged = false;
        }   

        if (brakeEngaged) {
            m_drivetrainSubsystem.setControl(brake); 
            return;
        }


        if (slowModeSupplier != null) {
            boolean currentSlowMode = slowModeSupplier.getAsBoolean();
            if (currentSlowMode != slowMode) {
                xLimiter.reset(inputs.x);
                yLimiter.reset(inputs.y);
            }
            slowMode = currentSlowMode;
            if (slowMode) {
                inputs.x = xLimiter.calculate(inputs.x);
                inputs.y = yLimiter.calculate(inputs.y);
            }
        }

        if (m_drivetrainSubsystem.isPigeonConnected()) {
            robotCentricControl.withFieldCentricSpeeds(inputs.x, inputs.y, rotationSpeed, gyroRotation);
        } else {
            robotCentricControl.withRobotCentricSpeeds(inputs.x, inputs.y, rotationSpeed);
        }

        m_drivetrainSubsystem.setControl(robotCentricControl.withCenterOfRotation(
            m_drivetrainSubsystem.getDriverCenterOfRotation()
        ));
    }

    @Override
    public void end(boolean interrupted) {}
}
