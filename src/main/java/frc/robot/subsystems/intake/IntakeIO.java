package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the intake subsystem */
public interface IntakeIO {
  /** stores the inputs data of intake */
  @AutoLog
  class IntakeInputs {
    public boolean noteDetected;
  }

  /** runs a voltage on this intake */
  default void runIntakeVoltage(double volts) {}

  /** runs a velocity on this intake */
  default void runIntakeVelocity(double velocity) {}

  /** stop the intake */
  default void stop() {}

  /** update the sensor inputs */
  void updateInputs(IntakeInputs inputs);
}
