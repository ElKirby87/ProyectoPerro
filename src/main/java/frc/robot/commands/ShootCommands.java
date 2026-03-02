// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.mecanismos.Conveyor;
import frc.robot.subsystems.mecanismos.LowShoot;
import frc.robot.subsystems.mecanismos.Shooter;

/** Add your docs here. */
public class ShootCommands {
  private ShootCommands() {}

  public static Command shoot(Shooter shooter, Conveyor conveyor, LowShoot lowShoot) {
    return Commands.parallel(
        shooter.moverse(),
        new SequentialCommandGroup(
            new WaitCommand(0.75), new ParallelCommandGroup(lowShoot.sigue(), conveyor.rcond())));
  }

  public static Command shoot(
      Shooter shooter, Conveyor conveyor, LowShoot lowShoot, double timeout) {
    return Commands.parallel(
            shooter.moverse(),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(lowShoot.sigue(), conveyor.rcond())))
        .withTimeout(timeout);
  }
}
