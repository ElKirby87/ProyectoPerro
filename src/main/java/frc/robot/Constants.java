// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class PositionConstants {
    public static final double desiredDistanceFromHub = 3.0;

    public static Pose2d HUB2_POSE = new Pose2d(4.65, 4.0, new Rotation2d()); // Blue hub
    public static Pose2d HUB1_POSE = new Pose2d(12.0, 4.0, new Rotation2d()); // Red hub

    public static final boolean HUB2_INVERT_FACING = true;
  }

  public static class DriveConstants {
    public static final double DEADBAND = 0.2;
    public static final double ANGLE_KP = 20.0;
    public static final double ANGLE_KD = 0.01;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0;
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    public static final double radialKp = 5.0;
    public static final double radialKd = 0.01;
    public static final double radialKi = 0.0;

    public static final double radialToleranceMeters = 0.05;
    public static final double angleToleranceRadians = Math.toRadians(3.0);
  }
}
