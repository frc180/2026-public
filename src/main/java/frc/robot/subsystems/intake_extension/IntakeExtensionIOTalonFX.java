package frc.robot.subsystems.intake_extension;

import static frc.robot.util.StatusSignals.trackSignal;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.StatusSignals;

public class IntakeExtensionIOTalonFX implements IntakeExtensionIO {

    private TalonFX extensionMotor;
    private MotionMagicVoltage motionMagicPosition;
    private DynamicMotionMagicVoltage dynamicPosition;
    private CoastOut coast = new CoastOut();

    final StatusSignal<Angle> motorPosition;
    final StatusSignal<Double> targetPosition;
    final StatusSignal<Current> supplyCurrent;

    // Simulation

    private TalonFXSimState talonFXSim;

    private static final double kGearRatio = 6;
    private final DCMotorSim motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    public IntakeExtensionIOTalonFX() {
        extensionMotor = new TalonFX(Constants.INTAKE_EXTENSION, Constants.CANIVORE_BUS);
        motionMagicPosition = new MotionMagicVoltage(0);
        dynamicPosition = new DynamicMotionMagicVoltage(0, IntakeExtensionSubsystem.MAX_VELOCITY, IntakeExtensionSubsystem.MAX_ACCELERATION);
        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();
        if (Robot.isReal()) {
            slot0.kP = 26;
        } else {
            slot0.kP = 5.0; // Placeholder
        }
        config.Slot0 = slot0;
        config.MotionMagic.MotionMagicAcceleration = IntakeExtensionSubsystem.MAX_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = IntakeExtensionSubsystem.MAX_VELOCITY;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40.0;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = kGearRatio;

        extensionMotor.getConfigurator().apply(config);

        motorPosition = trackSignal(extensionMotor.getPosition());
        targetPosition = trackSignal(extensionMotor.getClosedLoopReference());
        supplyCurrent = trackSignal(extensionMotor.getSupplyCurrent());

        StatusSignals.registerAlert("Extension", motorPosition);

        if (Robot.isReal()) return;

        talonFXSim = extensionMotor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void update(Inputs inputs) {
        inputs.position = motorPosition.getValueAsDouble();
        inputs.targetPosition = targetPosition.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        if (Robot.isReal()) return;

        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = talonFXSim.getMotorVoltageMeasure();
        motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        motorSimModel.update(0.020);

        talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(kGearRatio));
    }

    @Override
    public void setPosition(double position) {
        extensionMotor.setControl(motionMagicPosition.withPosition(position));
    }

    @Override
    public void setPosition(double position, double maxVel, double maxAccel) {
        extensionMotor.setControl(
            dynamicPosition.withPosition(position)
                           .withVelocity(maxVel)
                           .withAcceleration(maxAccel)
        );
    }

    @Override
    public void resetPosition(double position) {
        extensionMotor.setPosition(position);
        if (talonFXSim != null) {
            talonFXSim.setRawRotorPosition(position);
        }
    }

    @Override
    public void stop() {
        extensionMotor.set(0);
    }

    @Override
    public void setPower(double power) {
        extensionMotor.set(power);
    }

    @Override
    public void coast() {
       extensionMotor.setControl(coast);
    }
}
