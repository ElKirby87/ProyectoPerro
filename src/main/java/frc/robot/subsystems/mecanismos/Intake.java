// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mecanismos;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  TalonFX m_motor = new TalonFX(Constants.IntakeConstants.kIntakeMotorId);
  /** Creates a new Intake. */
  public Intake() {}

  public void conduce(Boolean isOn) {
    if (isOn) {
      m_motor.set(-Constants.IntakeConstants.intakeSpeed);
    } else {
      m_motor.set(Constants.IntakeConstants.intakeSpeed);
    }
  }

  public void reposo() {
    m_motor.set(0);
  }

  public Command autointake(double seconds, Boolean isOn) {
    return run(() -> {
          conduce(isOn);
        })
        .handleInterrupt(
            () -> {
              reposo();
            })
        .finallyDo(
            () -> {
              reposo();
            })
        .withTimeout(seconds);
  }

  public Command moverse(Boolean isOn) {
    return run(() -> {
          conduce(isOn);
        })
        .handleInterrupt(
            () -> {
              reposo();
            })
        .finallyDo(
            () -> {
              reposo();
            });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
