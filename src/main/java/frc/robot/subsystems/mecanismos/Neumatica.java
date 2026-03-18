// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mecanismos;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Neumatica extends SubsystemBase {
  private final Compressor compressor;
  private final Solenoid Intake1;
  private final Solenoid Intake2;

  /** Creates a new Neumatica. */
  public Neumatica() {
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();
    Intake1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Intake2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  }

  public void changeIntakeState(boolean state) {
    Intake1.set(state);
  }

  public void toggleIntakeState() {
    Intake1.toggle();
  }

  public void changeIntake2State(boolean state) {
    Intake2.set(state);
  }

  public void toggleIntake2State() {
    Intake2.toggle();
  }

  public Command toggleIntakeCommand() {
    return new Command() {
      @Override
      public void execute() {
        toggleIntakeState();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command toggleIntake2Command() {
    return new Command() {
      @Override
      public void execute() {
        toggleIntake2State();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command SacaIntake() {
    return new ParallelCommandGroup(toggleIntakeCommand(), toggleIntake2Command());
  }

  public Command enableCompressor() {
    return new Command() {
      @Override
      public void execute() {
        compressor.enableDigital();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
