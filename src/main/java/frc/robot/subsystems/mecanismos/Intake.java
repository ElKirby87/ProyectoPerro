// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mecanismos;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  SparkMax motor = new SparkMax(25, MotorType.kBrushed);
  /** Creates a new Intake. */
  public Intake() {}

  public void conduce() {
    motor.set(.5);
  }

  public void reposo() {
    motor.set(0);
  }

  public Command autointake() {
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

  public Command moverse() {
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
