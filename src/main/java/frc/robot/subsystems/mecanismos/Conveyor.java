// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mecanismos;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  TalonFX motor1 = new TalonFX(18);

  /** Creates a new Conveyor. */
  public Conveyor() {}

  public void reversa() {
    motor1.set(.1);
  }

  public void reposo() {
    motor1.set(0);
  }

  public Command rcond() {
    return run(() -> {
          reversa();
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

  public Command autoconv() {
    return run(() -> {
          reversa();
        })
        .handleInterrupt(
            () -> {
              reposo();
            })
        .finallyDo(
            () -> {
              reposo();
            })
        .withTimeout(5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
