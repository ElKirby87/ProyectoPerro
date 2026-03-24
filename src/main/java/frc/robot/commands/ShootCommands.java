// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.mecanismos.Conveyor;
import frc.robot.subsystems.mecanismos.Intake;
import frc.robot.subsystems.mecanismos.LowShoot;
import frc.robot.subsystems.mecanismos.Shooter;

/** Add your docs here. */
public class ShootCommands {
  private ShootCommands() {}

  public static Command shoot(
      Shooter shooter, Conveyor conveyor, LowShoot lowShoot, Intake intake) {
    return Commands.parallel(
        shooter.moverse(),
        new SequentialCommandGroup(
            new WaitCommand(Constants.CommandsConstants.shootWaitCommandSeconds),
            new ParallelCommandGroup(lowShoot.sigue(), conveyor.rcond(), intake.moverse(true))));
  }

  /*
  public static Command Smartshoot(
      Shooter shooter, Conveyor conveyor, LowShoot lowShoot, Intake intake) {
    return Commands.parallel(
        shooter.Activar(),
        new SequentialCommandGroup(
            new WaitCommand(Constants.CommandsConstants.shootWaitCommandSeconds),
            new ParallelCommandGroup(lowShoot.sigue(), conveyor.rcond(), intake.moverse(true))));
  }
            */

  public static Command shoot(
      Shooter shooter, Conveyor conveyor, LowShoot lowShoot, Intake intake, double timeout) {
    return Commands.parallel(
            shooter.moverse(),
            new SequentialCommandGroup(
                new WaitCommand(Constants.CommandsConstants.shootWaitCommandSeconds),
                new ParallelCommandGroup(lowShoot.sigue(), conveyor.rcond(), intake.moverse(true))))
        .withTimeout(timeout);
  }

  public static Command Conveyor(Conveyor conveyor, LowShoot lowShoot, Intake intake) {
    return Commands.parallel(lowShoot.sigue(), conveyor.rcond(), intake.moverse(true));
  }

  public static Command Conveyor(
      Conveyor conveyor, LowShoot lowShoot, Intake intake, double seconds) {
    return Commands.parallel(lowShoot.sigue(), conveyor.rcond(), intake.moverse(true))
        .withTimeout(seconds);
  }

  public static Command Shoot(Shooter shooter) {
    return Commands.run(() -> shooter.moverse(), shooter);
  }
}
