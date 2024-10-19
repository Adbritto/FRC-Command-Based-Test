package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * hardware implementation of IntakeIO using krakenX60 with talon fx and a beam breaker sensor
 */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeKraken = new TalonFX(0);
  private final DigitalInput intakeBeamBreaker = new DigitalInput(4);
  
  private final MotionMagicVelocityVoltage mVoltage = new MotionMagicVelocityVoltage(0);

  private final StatusSignal<Double> intakeVelocity = intakeKraken.getVelocity();
  private final StatusSignal<Double> intakeAppliedVolts = intakeKraken.getMotorVoltage();
  private final StatusSignal<Double> intakeCurrent = intakeKraken.getSupplyCurrent();

  public IntakeIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    // set current limits
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40.0;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25;
    slot0Configs.kV = 0.12;
    slot0Configs.kA = 0.01;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400;
    motionMagicConfigs.MotionMagicJerk = 4000;

    // apply configs to motor
    intakeKraken.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // detect the note with the beam breaker and store it in "inputs"
    // if the note is present, the beam breaker is blocked and will return "false"
    inputs.noteDetected = !intakeBeamBreaker.get();
  }

  @Override
  public void runIntakeVoltage(double volts) {
    // run the voltage on the intake kraken
    intakeKraken.setControl(new VoltageOut(volts / RobotController.getBatteryVoltage()));
  }

  @Override
  public void runIntakeVelocity(double velocity) {
    // run velocity on intake kraken
    intakeKraken.setControl(mVoltage.withVelocity(velocity));
  }

  @Override
  public void stop() {
    intakeKraken.stopMotor();
  }
}
