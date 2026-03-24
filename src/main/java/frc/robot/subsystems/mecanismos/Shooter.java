// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mecanismos;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX motor = new TalonFX(Constants.ShooterConstants.kShooterMotorId);

  private final VoltageOut shooterVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  Boolean isFollowing = true;
  double meters = 0.0;

  public Shooter() {}

  public void conduce() {
    if (isFollowing) {
      motor.setControl(
          shooterVoltageRequest.withOutput(Constants.ShooterConstants.shooterSpeed * 12));
    } else {
      motor.setControl(
          shooterVoltageRequest.withOutput(
              meters * (Constants.ShooterConstants.speedPerMeter * 12)));
    }
  }

  public void SmartShoot() {
    motor.set(meters * Constants.ShooterConstants.speedPerMeter);
  }

  public void SetMeters(double meters) {
    this.meters = meters;
  }

  public void reposo() {
    motor.set(0);
  }

  public void setFollow(Boolean isFollowing) {
    this.isFollowing = isFollowing;
  }

  public Command autoshoot() {
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

  public Command Activar() {
    return run(() -> {
          SmartShoot();
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
    SmartDashboard.putNumber("Distance", meters);
  }
}
