package frc.robot.subsystems.turret;

import static frc.robot.util.StatusSignals.trackSignal;
import java.util.List;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.StatusSignals;

public class TurretIOTalonFX implements TurretIO {

    static final double ROTOR_TO_MECHANISM_RATIO = 31.2;

    // These values are obtained by first zeroing out these values, pushing code,
    // rebooting the robot, then reading the value of absolutePositionRot & secondaryAbsolutePositionRot
    static final double ENCODER_A_ZERO_ROT = 0.380127;
    static final double ENCODER_B_ZERO_ROT = 0.048584;

    final TalonFX motor;
    final TalonFXConfiguration fxConfig;
    final CANcoder encoderA, encoderB;
    final PositionVoltage positionControl = new PositionVoltage(0);
    final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
    final CoastOut coastControl = new CoastOut();

    final StatusSignal<Angle> motorPosition, encoderAPosition, encoderBPosition;
    final StatusSignal<AngularVelocity> motorVelocity;
    final StatusSignal<Double> motorTarget;
    final StatusSignal<Current> supplyCurrent;

    // Simulation

    TalonFXSimState talonFXSim;
    CANcoderSimState cancoderSim;
    SingleJointedArmSim armSimModel;

    public TurretIOTalonFX() {
        motor = new TalonFX(Constants.TURRET, Constants.CANIVORE_BUS);
        encoderA = new CANcoder(Constants.TURRET_ENCODER_A, Constants.CANIVORE_BUS);
        encoderB = new CANcoder(Constants.TURRET_ENCODER_B, Constants.CANIVORE_BUS);

        fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 50;
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        fxConfig.Feedback.FeedbackRemoteSensorID = encoderA.getDeviceID();
        fxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(TurretSubsystem.PHYSICAL_MAX_POSITION_DEGREES);
        fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(TurretSubsystem.PHSYICAL_MIN_POSITION_DEGREES);
        fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        fxConfig.Feedback.SensorToMechanismRatio = ROTOR_TO_MECHANISM_RATIO;

        
        if (Robot.isReal()) {
            fxConfig.Slot0.kP = 120;
            fxConfig.Slot0.kV = 3.6;
        } else {
            fxConfig.Slot0.kP = 160;
            fxConfig.Slot0.kV = 3.8;
        }

        // currently unused motion magic
        fxConfig.MotionMagic.MotionMagicAcceleration = 12.5;
        fxConfig.MotionMagic.MotionMagicCruiseVelocity = 20;
        motor.getConfigurator().apply(fxConfig);

        CANcoderConfiguration ccAConfig = new CANcoderConfiguration();
        ccAConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        ccAConfig.MagnetSensor.MagnetOffset = -ENCODER_A_ZERO_ROT;
        encoderA.getConfigurator().apply(ccAConfig);
        
        CANcoderConfiguration ccBConfig = new CANcoderConfiguration();
        ccBConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        ccBConfig.MagnetSensor.MagnetOffset = -ENCODER_B_ZERO_ROT;
        encoderB.getConfigurator().apply(ccBConfig);

        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorTarget = trackSignal(motor.getClosedLoopReference());
        supplyCurrent = trackSignal(motor.getSupplyCurrent());
        encoderAPosition = trackSignal(encoderA.getPosition());
        encoderBPosition = trackSignal(encoderB.getPosition());

        StatusSignals.registerAlert("Turret Motor", motorTarget);
        StatusSignals.registerAlert("Turret CANcoder A", encoderAPosition);
        StatusSignals.registerAlert("Turret CANcoder B", encoderBPosition);

        if (Robot.isReal()) return;

        talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        cancoderSim = encoderA.getSimState();

        armSimModel = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            ROTOR_TO_MECHANISM_RATIO,
            0.001,
            0.001,
            Units.degreesToRadians(TurretSubsystem.MIN_POSITION_DEGREES),
            Units.degreesToRadians(TurretSubsystem.MAX_POSITION_DEGREES),
            false,
            Units.degreesToRadians(50)
        );
    }

    // Provides list of StatusSignals to be checked at higher speed by StatusSignalLoop.
    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        // Only read position and velocity at 250 hz
        return List.of(motorPosition, motorVelocity);
    }

    // Called at 250hz by StatusSignalLoop
    @Override
    public void fastUpdate(FastTurretInputs inputs) {
        double compensatedPosition = BaseStatusSignal.getLatencyCompensatedValueAsDouble(motorPosition, motorVelocity);
        inputs.positionDegrees = Units.rotationsToDegrees(compensatedPosition);
        inputs.velocityRPS = motorVelocity.getValueAsDouble();
    }

    @Override
    public void update(TurretInputs inputs) {
        inputs.absolutePositionRot = encoderAPosition.getValueAsDouble();
        inputs.secondaryAbsolutePositionRot = encoderBPosition.getValueAsDouble();
        inputs.absolutePositionDegrees = Units.rotationsToDegrees(inputs.absolutePositionRot);
        inputs.secondaryAbsolutePositionDegrees = Units.rotationsToDegrees(inputs.secondaryAbsolutePositionRot);
        inputs.targetDegrees = Units.rotationsToDegrees(motorTarget.getValueAsDouble());
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        armSimModel.setInputVoltage(motorVoltage.in(Volts));
        armSimModel.update(0.020); // assume 20 ms loop time

        double angle = Units.radiansToRotations(armSimModel.getAngleRads());
        double velocity = Units.radiansToRotations(armSimModel.getVelocityRadPerSec());

        // apply the new rotor position and velocity to the TalonFX;
        talonFXSim.setRawRotorPosition(angle * ROTOR_TO_MECHANISM_RATIO);
        talonFXSim.setRotorVelocity(velocity * ROTOR_TO_MECHANISM_RATIO);
    }

    @Override
    public void setSpeed(double power) {
        motor.setVoltage(power * 12);
    }

    @Override
    public void setPositionTarget(double degrees) {
        setPositionTarget(degrees, 0);
    }

    @Override
    public void setPositionTarget(double degrees, double rotPerSec) {
        setPositionTarget(degrees, rotPerSec, 0);
    }

    @Override
    public void setPositionTarget(double degrees, double rotPerSec, int slot) {
        if (TurretSubsystem.ROBORIO_PROFILING) {
            motor.setControl(
                positionControl.withPosition(Units.degreesToRotations(degrees))
                                .withVelocity(rotPerSec)
                                .withSlot(slot)
            );
        } else {
            motor.setControl(
                motionMagicControl.withPosition(Units.degreesToRotations(degrees))
            );
        }
    }

    @Override
    public void resetPosition(double absoluteDegrees) {
        motor.setPosition(Units.degreesToRotations(absoluteDegrees));
    }

    @Override
    public void coast() {
        motor.setControl(coastControl);
    }

    @NotLogged
    @Override
    public boolean enableSoftwareLimits() {
        fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        return motor.getConfigurator().apply(fxConfig).isOK();
    }
}