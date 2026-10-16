package frc.robot.subsystems.kicker;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.StatusSignals;
import frc.robot.util.simulation.SimUtils;

public class KickerIOTalonFX implements KickerIO {

    private final TalonFX kickerMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    final VelocityDutyCycle bangBang = new VelocityDutyCycle(0).withSlot(1);
    final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withSlot(2);

    StatusSignal<Voltage> motorVoltage;
    StatusSignal<AngularVelocity> motorVelocity;
    StatusSignal<Current> supplyCurrent, statorCurrent;

    // Simulation variables

    TalonFXSimState simState;
    private static final double kGearRatio = 21.0 / 20.0; // 15.0 / 14.0;

    private final DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.0005, kGearRatio
        ),
        DCMotor.getKrakenX60Foc(1)
    );


    public KickerIOTalonFX() {
        kickerMotor = new TalonFX(Constants.KICKER, Constants.CANIVORE_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 0.5;
        config.Slot0.kV = 0.137;
    
        config.Slot1.kP = 99999;

        config.Slot2.kP = 0;
        config.Slot2.kS = 0;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = kGearRatio;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        applyConfigWithRetry(config, "kicker motor");

        motorVoltage = trackSignal(kickerMotor.getMotorVoltage());
        motorVelocity = trackSignal(kickerMotor.getVelocity());
        supplyCurrent = trackSignal(kickerMotor.getSupplyCurrent());
        statorCurrent = trackSignal(kickerMotor.getStatorCurrent());

        StatusSignals.registerAlert("Kicker", motorVelocity);

        if (Robot.isReal()) return;

        simState = kickerMotor.getSimState();
        simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    private void applyConfigWithRetry(TalonFXConfiguration config, String name) {
        final int attempts = 5;
        StatusCode status = null;

        for (int i = 0; i < attempts; i++) {
            status = kickerMotor.getConfigurator().apply(config);
            if (status.isOK()) {
                return;
            }
        }

        System.err.println("Failed to apply " + name + " config after "
                + attempts + " attempts: " + status);
    }

    @Override
    public void update(KickerInputs inputs) {
        inputs.appliedVolts = motorVoltage.getValueAsDouble();
        inputs.velocityRps = motorVelocity.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        if (SimUtils.justLaunchedFuel) {
            double simVelocity = motorSim.getAngularVelocity().in(RadiansPerSecond);
            double reduction = 0.5 + Math.random() * 0.04;
            motorSim.setAngularVelocity(simVelocity * reduction);
            SimUtils.justLaunchedFuel = false;
        } else {
            var motorVoltage = simState.getMotorVoltageMeasure();
            motorSim.setInputVoltage(motorVoltage.in(Volts));
            motorSim.update(0.020); // 20 ms
        }

        simState.setRotorVelocity(motorSim.getAngularVelocity().times(kGearRatio));
    }

    @Override
    public void setVoltage(double volts) {
        kickerMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(double rps) {
        kickerMotor.setControl(velocityVoltage.withVelocity(rps));
    }

    @Override
    public void setVelocityTorqueCurrent(double rps) {
        kickerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(rps));
    }
}
