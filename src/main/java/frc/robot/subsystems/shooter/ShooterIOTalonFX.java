package frc.robot.subsystems.shooter;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.StatusSignals;
import frc.robot.util.simulation.SimUtils;

@Logged
public class ShooterIOTalonFX implements ShooterIO {

    static final boolean NO_FOLLOW_MODE = true;
    static final boolean TORQUECURRENT_MODE = false;
    static final boolean SYNC_SLOTS_TUNING = false;

    static final double GEAR_RATIO = 19.0 / 18.0;

    final TalonFX motor;
    final TalonFX followerMotorA;
    final CANrange canrange;
    final VelocityVoltage velocityVoltage;
    final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    final TalonFXConfiguration config, followerConfig;

    final StatusSignal<AngularVelocity> velocitySignal;
    final StatusSignal<Voltage> voltageSignal;
    final StatusSignal<Current> supplyCurrent, followerSupplyCurrent, statorCurrent;
    final StatusSignal<Distance> distanceSignal;

    final Alert configAlert = new Alert("Shooter config not applied", AlertType.kError);

    // Simulation variables

    TalonFXSimState simState;
    CANrangeSimState canrangeSimState;

    private static final double kGearRatio = 1.0; 
    private final FlywheelSim motorSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), 0.0026879 * 2, kGearRatio 
        ),
        DCMotor.getKrakenX60Foc(2)
    );


    public ShooterIOTalonFX() {
        motor = new TalonFX(Constants.SHOOTER_PRIMARY, Constants.CANIVORE_BUS); 
        followerMotorA = new TalonFX(Constants.SHOOTER_FOLLOWER_A, Constants.CANIVORE_BUS);
        
        velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);
        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withSlot(2);

        config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // VelocityVoltage
        config.Slot0.kP = 0.57;
        config.Slot0.kV = 0.124;

        // VelocityTorqueCurrentFOC values
        config.Slot2.kP = 10;
        config.Slot2.kS = 3.5;
    
        motor.getConfigurator().apply(config);

        followerConfig = config.clone();
        if (NO_FOLLOW_MODE) {
            followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Opposite of primary
        }
        followerMotorA.getConfigurator().apply(followerConfig);

        if (!NO_FOLLOW_MODE) {
            Follower followRequest = new Follower(Constants.SHOOTER_PRIMARY, MotorAlignmentValue.Opposed);
            followerMotorA.setControl(followRequest);
        }


        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
        rangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        canrange = new CANrange(Constants.SHOOTER_CANRANGE, Constants.CANIVORE_BUS);
        canrange.getConfigurator().apply(rangeConfig);

        
        velocitySignal = trackSignal(motor.getVelocity());
        voltageSignal = trackSignal(motor.getMotorVoltage());
        supplyCurrent = trackSignal(motor.getSupplyCurrent());
        followerSupplyCurrent = trackSignal(followerMotorA.getSupplyCurrent());
        statorCurrent = trackSignal((motor.getStatorCurrent()));
        distanceSignal = trackSignal(canrange.getDistance());

        voltageSignal.setUpdateFrequency(100);
        
        StatusSignals.registerAlert("Shooter Primary", velocitySignal);
        StatusSignals.registerAlert("Shooter Follower", followerSupplyCurrent);
        StatusSignals.registerAlert("Shooter CANrange", distanceSignal);

        if (SYNC_SLOTS_TUNING) {
            SmartDashboard.putNumber("Shooter/kP", 0);
            SmartDashboard.putNumber("Shooter/kV", 0);
            SmartDashboard.putNumber("Shooter/kS", 0);
        }

        if (Robot.isReal()) return;
        
        simState = motor.getSimState();
        simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        simState.Orientation = ChassisReference.Clockwise_Positive;

        canrangeSimState = canrange.getSimState();
    }

    @NotLogged
    double tuningPrevKp = 99;
    @NotLogged
    double tuningPrevKv = 99;
    @NotLogged
    double tuningPrevKs = 99;

    @Override
    public void update(ShooterInputs inputs) {
        inputs.velocityRPS = velocitySignal.getValueAsDouble();
        inputs.appliedVoltage = voltageSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.supplyCurrentFollower = followerSupplyCurrent.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.fuelDistance = distanceSignal.getValueAsDouble();
        inputs.canrangeConnected = distanceSignal.getTimestamp().getLatency() <= 0.5;

        if (SYNC_SLOTS_TUNING) {
            double kP = SmartDashboard.getNumber("Shooter/kP", 0);
            double kV = SmartDashboard.getNumber("Shooter/kV", 0);
            double kS = SmartDashboard.getNumber("Shooter/kS", 0);
            if (kP != tuningPrevKp || kV != tuningPrevKv || kS != tuningPrevKs) {
                
                for (TalonFXConfiguration config : new TalonFXConfiguration[] {config, followerConfig}) {
                    config.Slot0.kP = kP;
                    config.Slot0.kV = kV;
                    config.Slot0.kS = kS;

                    config.Slot2.kP = kP;
                    config.Slot2.kV = kV;
                    config.Slot2.kS = kS;
                }

                if (motor.getConfigurator().apply(config).isOK() && followerMotorA.getConfigurator().apply(followerConfig).isOK()) {
                    tuningPrevKp = kP;
                    tuningPrevKv = kV;
                    tuningPrevKs = kS;
                    configAlert.set(false);
                } else {
                    System.out.println("Failed to apply config during tuning");
                    configAlert.set(true);
                }
            }

        }
    }

    @Override
    public void simulationPeriodic() {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVolts = simState.getMotorVoltageMeasure().in(Volts);
        double simVelocity = motorSim.getAngularVelocity().in(RadiansPerSecond);

        if (SimUtils.justLaunchedFuel) {
            // Add a loss of speed when shooting a fuel
            double reduction = 0.86 + Math.random() * 0.04; // 0.86 to 0.90
            motorSim.setAngularVelocity(simVelocity * reduction);
        } else if (motorVolts == 0 && simVelocity > 0) {
            // Sim flywheel is stopping nearly instantly when setting volts to 0.
            // In reality it takes some time to slow down due to friction.
            // Linear decay: constant deceleration in rad/s per tick (20ms)
            motorSim.setAngularVelocity(Math.max(0, simVelocity - 3.0));
        } else {     
            motorSim.setInputVoltage(motorVolts);
            motorSim.update(0.020);
        }

        simState.setRotorVelocity(motorSim.getAngularVelocity().times(kGearRatio));

        canrangeSimState.setDistance(SimUtils.justLaunchedFuel ? 0.1 : 0.3);
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
        if (NO_FOLLOW_MODE) {
            followerMotorA.set(power);
        }
    }

    @Override
    public void setVelocityRPS(double velocity) {
        setVelocityRPS(velocity, 0);
    }

    @Override
    public void setVelocityRPS(double velocity, double feedforwardVolts) {
        setVelocityRPS(velocity, feedforwardVolts, 0);
    }

    @Override
    public void setVelocityRPS(double velocity, double feedforwardVolts, int slot) {
        ControlRequest request;
        if (TORQUECURRENT_MODE) {
            request = velocityTorqueCurrentFOC.withVelocity(velocity);
        } else {
            request = velocityVoltage.withVelocity(velocity)
                                        .withFeedForward(feedforwardVolts)
                                        .withSlot(slot);
        }

        motor.setControl(request);
        if (NO_FOLLOW_MODE) {
            followerMotorA.setControl(request);
        }
    }
}

