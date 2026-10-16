package frc.robot.subsystems.indexer;

import static frc.robot.util.StatusSignals.trackSignal;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.StatusSignals;

@Logged
public class IndexerIOTalonFX implements IndexerIO {

    private static final double kGearRatio = 84.0 / 9.0;

    private final TalonFX motor;
    private final TalonFX follower;
    private final CANrange canrange;

    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    final StatusSignal<AngularVelocity> motorVelocity, followerVelocity;
    final StatusSignal<Angle> motorPosition;
    final StatusSignal<Current> supplyCurrent, supplyCurrentFollower, statorCurrent;
    final StatusSignal<Distance> distanceSignal;
    final StatusSignal<Double> signalStengthSignal;

    // Sim variables

    private TalonFXSimState motorSimState = null;

    private final DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(2),
            0.0005,
            kGearRatio
        ),
        DCMotor.getKrakenX60Foc(2)
    );

    public IndexerIOTalonFX() {
        motor = new TalonFX(Constants.INDEXER_A, Constants.CANIVORE_BUS);
        follower = new TalonFX(Constants.INDEXER_B, Constants.CANIVORE_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = kGearRatio;

        config.Slot0.kP = 2.5;
        config.Slot0.kV = 1.13;

        motor.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

        follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Aligned));

        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
        rangeConfig.FovParams.FOVRangeX = 6.75;
        rangeConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        canrange = new CANrange(Constants.CANRANGE, Constants.CANIVORE_BUS);
        canrange.getConfigurator().apply(rangeConfig);

        motorVelocity = trackSignal(motor.getVelocity());
        motorPosition = trackSignal(motor.getPosition());
        followerVelocity = trackSignal(follower.getVelocity());
        supplyCurrent = trackSignal(motor.getSupplyCurrent());
        supplyCurrentFollower = trackSignal(follower.getSupplyCurrent());
        statorCurrent = trackSignal(motor.getStatorCurrent());

        distanceSignal = trackSignal(canrange.getDistance());
        signalStengthSignal = trackSignal(canrange.getSignalStrength());

        StatusSignals.registerAlert("Indexer Primary (" + motor.getDeviceID() + ")", motorVelocity);
        StatusSignals.registerAlert("Indexer Follower (" + follower.getDeviceID() + ")", followerVelocity);
        StatusSignals.registerAlert("Hopper CANrange", distanceSignal);

        if (Robot.isReal()) return;

        motorSimState = motor.getSimState();
        motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void update(IndexerInputs inputs) {
        inputs.velocityRPS = motorVelocity.getValueAsDouble();

        inputs.positionDegrees = Units.rotationsToDegrees(motorPosition.getValueAsDouble());
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.supplyCurrentFollower = supplyCurrent.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.hopperDistance = distanceSignal.getValueAsDouble();
        inputs.distanceStrength = signalStengthSignal.getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        if (Robot.isReal()) return;

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = motorSimState.getMotorVoltageMeasure();
        motorSim.setInputVoltage(motorVoltage.in(Volts));
        motorSim.update(0.020);

        motorSimState.setRotorVelocity(
            motorSim.getAngularVelocity().times(kGearRatio)
        );
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public void setPositionDegrees(double degrees) {
        motor.setControl(positionVoltage.withPosition(degrees / 360.0));
    }

    @Override
    public void setVelocity(double rps) {
        motor.setControl(velocityVoltage.withVelocity(rps));
    }
}
