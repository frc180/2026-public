package frc.robot.subsystems.prekicker;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.StatusSignals;

public class PrekickerIOTalonFX implements PrekickerIO {

    final TalonFX motor;
    final VoltageOut voltageOut = new VoltageOut(0);
    final VelocityVoltage velocityControl = new VelocityVoltage(0);

    static final double GEAR_RATIO = 54.0 / 30.0;

    final StatusSignal<AngularVelocity> motorVelocity;

    public PrekickerIOTalonFX() {
        motor = new TalonFX(Constants.PREKICKER, Constants.CANIVORE_BUS);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
        config.Slot0.kP = 0;
        motor.getConfigurator().apply(config);

        motorVelocity = trackSignal(motor.getVelocity());

        StatusSignals.registerAlert("Prekicker", motorVelocity);
    }

    @Override
    public void update(PrekickerInputs inputs) {
        inputs.velocityRPS = motorVelocity.getValueAsDouble();
    }

    @Override
    public void setPower(double power) {
        motor.setControl(voltageOut.withOutput(power * 12));
    }

    @Override
    public void setVelocity(double rps) {
        motor.setControl(velocityControl.withVelocity(rps));
    }    
}
