// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          // Use gyro-relative rotation raw (ignores invertHeading) so driver controls stay
          // consistent
          Rotation2d rawBase = drive.getGyroRelativeRotationRaw();
          Rotation2d baseRot = isFlipped ? rawBase.plus(new Rotation2d(Math.PI)) : rawBase;
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, baseRot));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              // Use gyro-relative rotation as measurement for angle control
              double omega =
                  angleController.calculate(
                      drive.getGyroRelativeRotationRaw().getRadians(),
                      rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              Rotation2d rawBase = drive.getGyroRelativeRotationRaw();
              Rotation2d baseRot = isFlipped ? rawBase.plus(new Rotation2d(Math.PI)) : rawBase;
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, baseRot));
            },
            drive)
        .beforeStarting(
            () -> angleController.reset(drive.getGyroRelativeRotationRaw().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(DriveConstants.FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * DriveConstants.FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(DriveConstants.WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  /**
   * Mantiene distancia objetivo respecto a un AprilTag (por id) y alinea el heading con el tag. El
   * piloto sólo controla la componente tangencial (strafe) mediante tangentialSupplier.
   *
   * <p>/** Mantiene distancia objetivo (PositionConstants.desiredDistanceFromHub) respecto a una
   * pose de hub y hace que el robot siempre mire hacia el centro del hub. El piloto sólo controla
   * la componente tangencial (strafe) mediante tangentialSupplier para "orbitar".
   *
   * <p>- Usa odometría (drive.getPose()). La visión, cuando detecta tags, ya actualiza la odometría
   * via drive.addVisionMeasurement, por lo que no es necesario que la cámara esté siempre visible
   * mientras el comando funcione.
   */
  public static Command driveToHub(Drive drive, DoubleSupplier tangentialSupplier) {

    final double desiredDistance = PositionConstants.desiredDistanceFromHub;

    PIDController radialPID =
        new PIDController(
            DriveConstants.radialKp,
            DriveConstants.radialKi,
            DriveConstants.radialKd); // tunear en sim/real

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Selecciona el hub según la alliance (solo selección de pose)
              Pose2d hubPose;
              boolean isHub2;
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                hubPose = PositionConstants.HUB2_POSE;
                isHub2 = true;
              } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                hubPose = PositionConstants.HUB1_POSE;
                isHub2 = false;
              } else {
                hubPose = PositionConstants.HUB1_POSE;
                isHub2 = false;
              }

              // Lectura de pose/gyro
              Pose2d robotPose = drive.getPose();
              Rotation2d robotYaw = drive.getGyroRelativeRotationRaw(); // RAW gyro frame
              // Vector desde robot hasta hub (campo)
              Translation2d vecField = hubPose.getTranslation().minus(robotPose.getTranslation());
              double dist = vecField.getNorm();
              if (dist < 1e-6) {
                drive.runVelocity(new ChassisSpeeds(0, 0, 0));
                return;
              }

              // Transformar vector al marco del robot (robot-relative)
              Translation2d vecRobot =
                  vecField.rotateBy(robotYaw.unaryMinus()); // ahora en robot frame

              // Unidad radial en robot frame apuntando hacia el hub
              Translation2d radialUnitRobot =
                  new Translation2d(vecRobot.getX() / dist, vecRobot.getY() / dist);
              // Tangencial unitario en robot frame
              Translation2d tangentialUnitRobot;
              if (isHub2) {
                tangentialUnitRobot =
                    new Translation2d(radialUnitRobot.getY(), -radialUnitRobot.getX());
              } else {
                tangentialUnitRobot =
                    new Translation2d(-radialUnitRobot.getY(), radialUnitRobot.getX());
              }

              // Radial control: PID sobre la magnitud de la distancia
              // PID.calculate(measurement, setpoint) => (set - meas) * kP ...
              double radialError = dist - desiredDistance; // >0 => estamos lejos
              double radialSpeed = 0.0;
              if (Math.abs(radialError) > DriveConstants.radialToleranceMeters) {
                // Solo ejecutar PID si estamos fuera de la tolerancia radial
                double rawRadialOutput = radialPID.calculate(dist, desiredDistance);
                // Convertir salida PID en velocidad (positivo => mover hacia el hub)
                radialSpeed =
                    MathUtil.clamp(
                        -rawRadialOutput,
                        -drive.getMaxLinearSpeedMetersPerSec(),
                        drive.getMaxLinearSpeedMetersPerSec());
              } else {
                // Dentro de tolerancia: evitar micro-correcciones/ruido y resetear integrador
                radialPID.reset();
                radialSpeed = 0.0;
              }
              Translation2d radialVelRobot = radialUnitRobot.times(radialSpeed);
              // Tangential control from driver (robot-relative)
              double tangentialInput =
                  MathUtil.applyDeadband(
                      -tangentialSupplier.getAsDouble(), DriveConstants.DEADBAND);
              tangentialInput = Math.copySign(tangentialInput * tangentialInput, tangentialInput);
              double tangentialSpeed = tangentialInput * drive.getMaxLinearSpeedMetersPerSec();
              Translation2d tangentialVelRobot = tangentialUnitRobot.times(tangentialSpeed);

              // Sum velocities in robot frame
              Translation2d linearRobot = radialVelRobot.plus(tangentialVelRobot);

              // Angular control: face the hub (compute desired yaw in field then convert to gyro
              // frame)
              double desiredYawRad =
                  Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());
              // if (isHub2 && PositionConstants.HUB2_INVERT_FACING) desiredYawRad += Math.PI;
              double desiredYawRel = desiredYawRad - drive.getGyroZero().getRadians();
              double currentYawRel = robotYaw.getRadians();
              // Wrapped angular error (-pi..pi)
              double rawAngleError =
                  Math.atan2(
                      Math.sin(desiredYawRel - currentYawRel),
                      Math.cos(desiredYawRel - currentYawRel));
              double omega = 0.0;
              if (Math.abs(rawAngleError) > DriveConstants.angleToleranceRadians) {
                // Only run angular PID if error exceeds tolerance
                omega = angleController.calculate(currentYawRel, desiredYawRel);
              } else {
                // Inside angular tolerance: stop rotating and reset controller to avoid wind-up
                angleController.reset(currentYawRel);
                omega = 0.0;
              }

              // Build ChassisSpeeds in robot frame directly (vx forward, vy left)
              ChassisSpeeds robotSpeeds =
                  new ChassisSpeeds(linearRobot.getX(), linearRobot.getY(), omega);
              /*
              if (isHub2) {
                robotSpeeds =
                    new ChassisSpeeds(
                        -robotSpeeds.vxMetersPerSecond,
                        -robotSpeeds.vyMetersPerSecond,
                        robotSpeeds.omegaRadiansPerSecond);
              }
                        */

              // Send robot-relative speeds directly
              drive.runVelocity(robotSpeeds);
            },
            drive)
        .beforeStarting(
            () -> {
              radialPID.reset();
              angleController.reset(drive.getGyroRelativeRotationRaw().getRadians());
            });
  }

  public static Command driveToHubWithTimeout(Drive drive, double seconds) {

    final double desiredDistance = PositionConstants.desiredDistanceFromHub;

    PIDController radialPID =
        new PIDController(
            DriveConstants.radialKp,
            DriveConstants.radialKi,
            DriveConstants.radialKd); // tunear en sim/real

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Selecciona el hub según la alliance (solo selección de pose)
              Pose2d hubPose;
              boolean isHub2;
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                hubPose = PositionConstants.HUB2_POSE;
                isHub2 = true;
              } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                hubPose = PositionConstants.HUB1_POSE;
                isHub2 = false;
              } else {
                hubPose = PositionConstants.HUB1_POSE;
                isHub2 = false;
              }

              // Lectura de pose/gyro
              Pose2d robotPose = drive.getPose();
              Rotation2d robotYaw = drive.getGyroRelativeRotationRaw(); // RAW gyro frame
              // Vector desde robot hasta hub (campo)
              Translation2d vecField = hubPose.getTranslation().minus(robotPose.getTranslation());
              double dist = vecField.getNorm();
              if (dist < 1e-6) {
                drive.runVelocity(new ChassisSpeeds(0, 0, 0));
                return;
              }

              // Transformar vector al marco del robot (robot-relative)
              Translation2d vecRobot =
                  vecField.rotateBy(robotYaw.unaryMinus()); // ahora en robot frame

              // Unidad radial en robot frame apuntando hacia el hub
              Translation2d radialUnitRobot =
                  new Translation2d(vecRobot.getX() / dist, vecRobot.getY() / dist);
              // Tangencial unitario en robot frame
              Translation2d tangentialUnitRobot;
              if (isHub2) {
                tangentialUnitRobot =
                    new Translation2d(radialUnitRobot.getY(), -radialUnitRobot.getX());
              } else {
                tangentialUnitRobot =
                    new Translation2d(-radialUnitRobot.getY(), radialUnitRobot.getX());
              }

              // Radial control: PID sobre la magnitud de la distancia
              // PID.calculate(measurement, setpoint) => (set - meas) * kP ...
              double radialError = dist - desiredDistance; // >0 => estamos lejos
              double radialSpeed = 0.0;
              if (Math.abs(radialError) > DriveConstants.radialToleranceMeters) {
                // Solo ejecutar PID si estamos fuera de la tolerancia radial
                double rawRadialOutput = radialPID.calculate(dist, desiredDistance);
                // Convertir salida PID en velocidad (positivo => mover hacia el hub)
                radialSpeed =
                    MathUtil.clamp(
                        -rawRadialOutput,
                        -drive.getMaxLinearSpeedMetersPerSec(),
                        drive.getMaxLinearSpeedMetersPerSec());
              } else {
                // Dentro de tolerancia: evitar micro-correcciones/ruido y resetear integrador
                radialPID.reset();
                radialSpeed = 0.0;
              }
              Translation2d radialVelRobot = radialUnitRobot.times(radialSpeed);
              // Tangential control from driver (robot-relative)
              double tangentialInput = MathUtil.applyDeadband(-0, DriveConstants.DEADBAND);
              tangentialInput = Math.copySign(tangentialInput * tangentialInput, tangentialInput);
              double tangentialSpeed = tangentialInput * drive.getMaxLinearSpeedMetersPerSec();
              Translation2d tangentialVelRobot = tangentialUnitRobot.times(tangentialSpeed);

              // Sum velocities in robot frame
              Translation2d linearRobot = radialVelRobot.plus(tangentialVelRobot);

              // Angular control: face the hub (compute desired yaw in field then convert to gyro
              // frame)
              double desiredYawRad =
                  Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());
              // if (isHub2 && PositionConstants.HUB2_INVERT_FACING) desiredYawRad += Math.PI;
              double desiredYawRel = desiredYawRad - drive.getGyroZero().getRadians();
              double currentYawRel = robotYaw.getRadians();
              // Wrapped angular error (-pi..pi)
              double rawAngleError =
                  Math.atan2(
                      Math.sin(desiredYawRel - currentYawRel),
                      Math.cos(desiredYawRel - currentYawRel));
              double omega = 0.0;
              if (Math.abs(rawAngleError) > DriveConstants.angleToleranceRadians) {
                // Only run angular PID if error exceeds tolerance
                omega = angleController.calculate(currentYawRel, desiredYawRel);
              } else {
                // Inside angular tolerance: stop rotating and reset controller to avoid wind-up
                angleController.reset(currentYawRel);
                omega = 0.0;
              }

              // Build ChassisSpeeds in robot frame directly (vx forward, vy left)
              ChassisSpeeds robotSpeeds =
                  new ChassisSpeeds(linearRobot.getX(), linearRobot.getY(), omega);
              /*
              if (isHub2) {
                robotSpeeds =
                    new ChassisSpeeds(
                        -robotSpeeds.vxMetersPerSecond,
                        -robotSpeeds.vyMetersPerSecond,
                        robotSpeeds.omegaRadiansPerSecond);
              }
                        */

              // Send robot-relative speeds directly
              drive.runVelocity(robotSpeeds);
            },
            drive)
        .beforeStarting(
            () -> {
              radialPID.reset();
              angleController.reset(drive.getGyroRelativeRotationRaw().getRadians());
            })
        .withTimeout(seconds);
  }

  /** Conveniencia: elegir hub por índice 1 o 2 (usa HUB1_POSE/HUB2_POSE). */
  /*
    public static Command driveToHub(Drive drive, int hubIndex, DoubleSupplier tangentialSupplier) {
      return driveToHub(drive, tangentialSupplier);
    }
  */
  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
