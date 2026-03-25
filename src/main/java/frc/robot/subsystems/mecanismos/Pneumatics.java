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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private final Compressor compressor;

  private final Solenoid compuerta = new Solenoid(PneumaticsModuleType.REVPH, 5);
  private final Solenoid brazoDerechaSolenoide = new Solenoid(PneumaticsModuleType.REVPH, 6);
  private final Solenoid brazoIzquierdaSolenoide = new Solenoid(PneumaticsModuleType.REVPH, 7);
  AnalogPotentiometer pressureSensore = new AnalogPotentiometer(0, 150, -25);
  private Boolean compressorActivated = true;

  public Pneumatics() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.disable();
    // compuerta.set(true);
  }

  public void setCompuerta(Boolean isOpen) {
    compuerta.set(isOpen);
  }

  public void toggleCompuerta() {
    compuerta.toggle();
  }

  public void estirarIntake() {
    brazoDerechaSolenoide.set(true);
    brazoIzquierdaSolenoide.set(true);
    compuerta.set(true);
  }

  public void retraerIntake() {
    brazoDerechaSolenoide.set(false);
    brazoIzquierdaSolenoide.set(false);
  }

  public void toggleSolenoidState() {
    brazoDerechaSolenoide.toggle();
    brazoIzquierdaSolenoide.toggle();
    // solenoid1.set(isOn);
    // solenoid2.set(isOn);
    // solenoid3.set(isOn);
  }

  public Command toggleCompuerta(Boolean isOn) {
    return new Command() {
      @Override
      public void execute() {
        if (isOn) {
          compuerta.set(true);
        } else {
          compuerta.set(false);
        }
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
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
    return Commands.run(
            () -> {
              retraerIntake();
            })
        .withTimeout(3)
        .finallyDo(
            () -> {
              compuerta.set(false);
            });
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

  public Command abrirCompuerta() {
    return new Command() {
      @Override
      public void execute() {
        toggleCompuerta();
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
