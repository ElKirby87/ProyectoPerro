// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mecanismos;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LowShoot extends SubsystemBase {
  /** Creates a new LowShoot. */
  TalonFX motor3 = new TalonFX(19);

  public LowShoot() {}

  public void conduce() {
    motor3.set(.5);
  }

  public void reposo() {
    motor3.set(0);
  }

  public Command autolowshoot() {
    return run(() -> {
          conduce();
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

  public Command sigue() {
    return run(() -> {
          conduce();
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
