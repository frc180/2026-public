package frc.robot.subsystems.hood;

import static frc.robot.util.StatusSignals.trackSignal;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.StatusSignals;

public class HoodIOTalonFX implements HoodIO {

    static final double ROTOR_TO_MECHANISM_RATIO = 384.0 / 25.0;

    final TalonFX motor;

    final StatusSignal<Angle> motorPosition;
    final StatusSignal<Current> supplyCurrent;
    final StatusSignal<Double> targetPosition;

    final PositionVoltage positionRequest = new PositionVoltage(0);
    final MotionMagicVoltage magicPositionRequest = new MotionMagicVoltage(0);

    // Simulation variables

    private TalonFXSimState talonFXSim;

    private final DCMotorSim motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), 0.001, ROTOR_TO_MECHANISM_RATIO
        ),
        DCMotor.getKrakenX44Foc(1)
    );

    public HoodIOTalonFX() {
        motor = new TalonFX(Constants.SHOOTER_HOOD, Constants.CANIVORE_BUS);

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 50;
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(HoodSubystem.MAX_DEGREES);
        fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(HoodSubystem.MIN_DEGREES);
        fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        fxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        fxConfig.Feedback.SensorToMechanismRatio = ROTOR_TO_MECHANISM_RATIO;

        if (Robot.isReal()) {
            fxConfig.Slot0.kP = 250;
            fxConfig.Slot0.kD = 1.6;
            fxConfig.Slot0.kV = 0.6;
            fxConfig.Slot0.kG = 0.25;
            fxConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        } else {
            fxConfig.Slot0.kP = 100;
        }
        fxConfig.MotionMagic.MotionMagicCruiseVelocity = 999;
        fxConfig.MotionMagic.MotionMagicAcceleration = 25;
        motor.getConfigurator().apply(fxConfig);

        motorPosition = trackSignal(motor.getPosition());
        targetPosition = trackSignal(motor.getClosedLoopReference());
        supplyCurrent = trackSignal(motor.getSupplyCurrent());

        StatusSignals.registerAlert("Hood Motor", motorPosition);

        if (Robot.isReal()) return;

        motorSimModel.setAngle(Units.degreesToRadians(HoodSubystem.START_DEGREES));

        talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    @Override
    public void update(HoodInputs inputs) {
        inputs.positionDegrees = Units.rotationsToDegrees(motorPosition.getValueAsDouble());
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.targetDegrees = Units.rotationsToDegrees(targetPosition.getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        if (Robot.isReal()) return;

        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();
        motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        motorSimModel.update(0.020);

        AngularVelocity velocity = motorSimModel.getAngularVelocity();
        Angle angle = motorSimModel.getAngularPosition();

        talonFXSim.setRotorVelocity(velocity.times(ROTOR_TO_MECHANISM_RATIO));
        talonFXSim.setRawRotorPosition(angle.times(ROTOR_TO_MECHANISM_RATIO));
    }

    @Override
    public void setAngle(double degrees) {
        setAngle(degrees, 0);
    }

    @Override
    public void setAngle(double degrees, double velocityRPS) {
        double position = Units.degreesToRotations(degrees);
        if (HoodSubystem.ROBORIO_PROFILING) {
            motor.setControl(positionRequest.withPosition(position).withVelocity(velocityRPS));
        } else {
            motor.setControl(magicPositionRequest.withPosition(position));
        }
    }

    @Override
    public void setPower(double power) {
        motor.setVoltage(power * 12);
    }

    @Override
    public void resetAngle(double degrees) {
        motor.setPosition(Units.degreesToRotations(degrees));
    }

    @Override
    public void resetAngleNonBlocking(double degrees) {
        motor.setPosition(Units.degreesToRotations(degrees), 0);
    }
}
