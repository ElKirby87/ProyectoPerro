// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mecanismos;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private final Compressor compressor;

  private final Solenoid solenoideClimb = new Solenoid(PneumaticsModuleType.REVPH, 5);
  private final Solenoid brazoDerechaSolenoide = new Solenoid(PneumaticsModuleType.REVPH, 6);
  private final Solenoid brazoIzquierdaSolenoide = new Solenoid(PneumaticsModuleType.REVPH, 7);
  AnalogPotentiometer pressureSensore = new AnalogPotentiometer(0, 150, -25);
  private Boolean compressorActivated = true;

  public Pneumatics() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableDigital();
  }

  public void estirarIntake() {
    brazoDerechaSolenoide.set(true);
    brazoIzquierdaSolenoide.set(true);
  }

  public void retraerIntake() {
    brazoDerechaSolenoide.set(false);
    brazoIzquierdaSolenoide.set(false);
  }

  public void toggleSolenoidState() {
    brazoDerechaSolenoide.toggle();
    brazoIzquierdaSolenoide.toggle();
    solenoideClimb.toggle();
    // solenoid1.set(isOn);
    // solenoid2.set(isOn);
    // solenoid3.set(isOn);
  }

  public Command EstirarIntake() {
    return new Command() {
      @Override
      public void execute() {
        estirarIntake();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command RetraerIntake() {
    return new Command() {
      @Override
      public void execute() {
        retraerIntake();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command toogleCommand() {
    return new Command() {
      @Override
      public void execute() {
        toggleSolenoidState();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command enableCompressor() {
    return new Command() {
      @Override
      public void execute() {
        if (compressorActivated) {
          compressor.disable();
        } else {
          compressor.enableDigital();
        }
        compressorActivated = !compressorActivated;
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
    SmartDashboard.putNumber("Presion", getPressure());
  }

  public double getPressure() {
    return pressureSensore.get();
  }
}
