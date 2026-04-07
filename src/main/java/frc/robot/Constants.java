// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(12);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
public static final class IntakeConstants {
  public static final int PIVOT_MOTOR_ID = 20;
  public static final boolean PIVOT_INVERTED = true;
  public static final int PIVOT_CURRENT_LIMIT = 30;

  // Gear ratio: 128 motor rotations : 1 pivot rotation
  public static final double PIVOT_GEAR_RATIO = 200.0;

  // Encoder conversion
  public static final double PIVOT_DEG_PER_MOTOR_ROT = 360.0 / PIVOT_GEAR_RATIO;   // 2.8125 deg
  public static final double PIVOT_DEG_PER_SEC_PER_RPM = PIVOT_DEG_PER_MOTOR_ROT / 60.0;

  // PID to start with
  public static final double PIVOT_kP = 0.035;
  public static final double PIVOT_kI = 0.0;
  public static final double PIVOT_kD = 0.0;

  // Output clamp for closed-loop
  public static final double PIVOT_MIN_OUTPUT = -1;
  public static final double PIVOT_MAX_OUTPUT = 1;

  // Tolerance
  public static final double PIVOT_TOLERANCE_DEG = 2.0;

  // Soft limits in pivot degrees from your manually-set zero
  public static final double PIVOT_MIN_ANGLE_DEG = -1000000.0;
  public static final double PIVOT_MAX_ANGLE_DEG = 100000.0;

  // Common setpoints
  public static final double PIVOT_STOW_ANGLE_DEG = 0.0;
  public static final double PIVOT_DEPLOY_ANGLE_DEG = 90.0;
  public static final double PIVOT_FEED_ANGLE_DEG = 30.0;

  // Manual jog power for manual override
  public static final double PIVOT_MANUAL_POWER = 0.60;
}

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 22;  // TODO set
    public static final boolean ROLLER_INVERTED = true;
    public static final int ROLLER_CURRENT_LIMIT = 50;
    public static final double INTAKE_POWER = 0.75;
    public static final double OUTTAKE_POWER = -0.75;
  }

  public static final class FeederConstants {
    public static final int INDEXER_MOTOR_ID = 13;   // TODO set
    public static final int SPINDEXER_MOTOR_ID = 15; // TODO set
    public static final boolean INDEXER_INVERTED = false;
    public static final boolean SPINDEXER_INVERTED = true;
    public static final int CURRENT_LIMIT = 35;
    public static final double FEED_POWER = 0.4;
    public static final double REVERSE_POWER = -0.20;
  }

  public static final class FlywheelConstants {
    public static final int FLYWHEEL_MOTOR_ID = 24; // TODO set
    public static final boolean FLYWHEEL_INVERTED = false;
    public static final int CURRENT_LIMIT = 50;
    public static final double SHOOT_POWER = 1.0;   // open-loop percent
    public static final double IDLE_POWER = 0.0;
  }

  public static final class TurretAimConstants {
  public static final double kP = 1.0;       // start here
  public static final double MIN_OUT = 0.8;  // to overcome stiction
  public static final double MAX_OUT = 1.3;  // keep it safe (matches your turret clamp)
  public static final double AIM_DEADBAND_DEG = 0.05;
}
public static final class ClimbConstants {
  public static final int CLIMB_MOTOR_ID = 27; // CHANGE THIS
  public static final boolean CLIMB_INVERTED = false;
  public static final int CURRENT_LIMIT = 40;
  public static final double UP_POWER = 0.5;
  public static final double DOWN_POWER = -0.5;
}
  public static final class VisionConstants
  {
    public static final String LIMELIGHT_NAME = "limelight-hub";

    // CAMERA POSITION ON ROBOT
    // +X forward, +Y left, +Z up
    public static final double CAM_X_METERS = 0.2667; // example: 10.5 in forward of robot center
    public static final double CAM_Y_METERS = 0.0;    // centered left/right
    public static final double CAM_Z_METERS = 0.501;  // about 19.7 in high

    // CAMERA MOUNTING ANGLES
    public static final double CAM_ROLL_DEG = 0.0;
    public static final double CAM_PITCH_DEG = 3.0;
    public static final double CAM_YAW_DEG = 0.0;

    // Reject bad measurements
    public static final double MAX_TAG_DISTANCE_METERS = 4.5;
    public static final double MAX_POSE_JUMP_METERS = 2.0;
    public static final double MAX_ROTATION_RATE_DEG_PER_SEC = 540.0;
  }
}
