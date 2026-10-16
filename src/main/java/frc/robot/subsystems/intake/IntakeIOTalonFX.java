package frc.robot.subsystems.intake;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import static edu.wpi.first.units.Units.*;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake_extension.IntakeExtensionSubsystem.Position;
import frc.robot.util.StatusSignals;
import frc.robot.util.simulation.SimUtils;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX intakeMotor, intakeSecondaryMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    final StatusSignal<Voltage> motorVoltage;
    final StatusSignal<AngularVelocity> motorVelocity;
    final StatusSignal<Current> supplyCurrent, secondarySupplyCurrent;

    public IntakeIOTalonFX() {
        intakeMotor = new TalonFX(Constants.INTAKE, Constants.CANIVORE_BUS);
        intakeSecondaryMotor = new TalonFX(Constants.INTAKE_SECONDARY, Constants.CANIVORE_BUS);

        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.kP = 0.8;
        config.Slot0.kV = 0.13;
        intakeMotor.getConfigurator().apply(config);

        intakeSecondaryMotor.getConfigurator().apply(config);
        intakeSecondaryMotor.setControl(new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        

        motorVoltage = trackSignal(intakeMotor.getMotorVoltage());
        motorVelocity = trackSignal(intakeMotor.getVelocity());
        supplyCurrent = trackSignal(intakeMotor.getSupplyCurrent());
        secondarySupplyCurrent = trackSignal(intakeSecondaryMotor.getSupplyCurrent());

        StatusSignals.registerAlert("Intake", motorVelocity);
        StatusSignals.registerAlert("Intake Secondary", secondarySupplyCurrent);

        if (Robot.isReal()) return;

        SimUtils.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            RobotContainer.instance.drivetrain.getDriveSim(),
            Inches.of(22),
            Inches.of(4),
            IntakeSide.FRONT,
            SimUtils.MAX_FUEL
        );
        SimUtils.intakeSimulation.stopIntake();
    }

    @Override
    public void update(IntakeInputs inputs) {
        inputs.velocityRps = motorVelocity.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        boolean intaking = intakeMotor.get() > 0;
        boolean intakeExtended = RobotContainer.instance.intakeExtension.atPosition(Position.EXTENDED);
        if (intaking && intakeExtended && SimUtils.getCombinedFuelCount() < SimUtils.MAX_FUEL) {
            SimUtils.intakeSimulation.startIntake();
        } else {
            SimUtils.intakeSimulation.stopIntake();
        }
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(double rps) {
        intakeMotor.setControl(velocityRequest.withVelocity(rps));
    }
}
