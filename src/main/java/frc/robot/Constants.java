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

    // Limit switches
  public static final int SLIDER_FORWARD_LIMIT_DIO = 0;
  public static final int SLIDER_REVERSE_LIMIT_DIO = 1;

  // Set these based on your wiring:
  // true = pressed reads false (common for normally closed wiring)
  // false = pressed reads true

  public static final boolean SLIDER_FORWARD_LIMIT_INVERTED = true;
  public static final boolean SLIDER_REVERSE_LIMIT_INVERTED = true;
  public static final int SLIDER_MOTOR_ID = 20;
  public static final boolean SLIDER_INVERTED = false;
  public static final int SLIDER_CURRENT_LIMIT = 50;

 // Rack-and-pinion slider geometry
  public static final double PINION_TOOTH_COUNT = 60.0;
  public static final double PINION_PITCH_DIAMETER_IN = 2.5;

  // Gear reduction: motor spins 8 times for 1 pinion rotation
  public static final double MOTOR_TO_PINION_RATIO = 8.0;

  // Linear slider travel in inches for one motor rotation
  public static final double SLIDER_POSITION_PER_MOTOR_ROTATION =
      (Math.PI * PINION_PITCH_DIAMETER_IN) / MOTOR_TO_PINION_RATIO; // 0.98175 in/rev

  // Slider velocity in inches/sec for 1 motor RPM
  public static final double SLIDER_VELOCITY_PER_RPM =
      SLIDER_POSITION_PER_MOTOR_ROTATION / 60.0; // 0.0163625 in/sec per RPM


  // Closed-loop tuning
  public static final double SLIDER_kP = 0.80;
  public static final double SLIDER_kI = 0.0;
  public static final double SLIDER_kD = 0.0;

  // Closed-loop output limits
  public static final double SLIDER_MIN_OUTPUT = -0.20;
  public static final double SLIDER_MAX_OUTPUT = 0.4;

  // Allowed slider travel range
  public static final double SLIDER_MIN_POSITION = -1000;
  public static final double SLIDER_MAX_POSITION = 1200;

  public static final double SLIDER_STEP_INCHES = 2.0;

  // Preset positions
  public static final double SLIDER_RETRACTED_POSITION = 0.0;
  public static final double SLIDER_EXTENDED_POSITION = 10.30;
  public static final double SLIDER_FEED_POSITION = 4.0;

  // Manual control
  public static final double SLIDER_MANUAL_POWER = 0.20;
  public static final double SLIDER_MANUAL_MAX_POWER = 0.20;

  // Position tolerance
  public static final double SLIDER_POSITION_TOLERANCE = .10;

  // Current-based hard-stop detection
  public static final double SLIDER_HARD_STOP_CURRENT_AMPS = 28.0;
  public static final double SLIDER_HARD_STOP_DEBOUNCE_SEC = 0.2;
}

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 22;  // TODO set
    public static final boolean ROLLER_INVERTED = true;
    public static final int ROLLER_CURRENT_LIMIT = 55;
    public static final double INTAKE_POWER = 0.90;
    public static final double OUTTAKE_POWER = -0.75;
  }

  public static final class FeederConstants {
    public static final int INDEXER_MOTOR_ID = 13;   // TODO set
    public static final int SPINDEXER_MOTOR_ID = 29; // TODO set
    public static final boolean INDEXER_INVERTED = false;
    public static final boolean SPINDEXER_INVERTED = false;
    public static final int CURRENT_LIMIT = 35;
    public static final double FEED_POWER = 0.7;
    public static final double REVERSE_POWER = -0.20;
  }

public static final class FlywheelConstants {
  public static final int FLYWHEEL_MOTOR_ID = 24;
  public static final boolean FLYWHEEL_INVERTED = false;

  public static final double NORMAL_SHOT_RPM = 5600.0;
  public static final double CLOSE_SHOT_RPM = 5200.0;
  public static final double FAR_SHOT_RPM = 6000.0;
    public static final double SHOOT_POWER = 0.90;
    public static final double IDLE_POWER = 0.90;


      public static final double SUPPLY_CURRENT_LIMIT = 70.0;
  public static final double STATOR_CURRENT_LIMIT = 110.0;
  
  
  public static final double RPM_TOLERANCE = 100.0;
  public static final double kS = 0.3;
  public static final double kV = 0.13;
  public static final double kA = 0.64;
  public static final double kP = 0.26;
  public static final double kI = 0.0;
  public static final double kD = 0.002;

  public static final double MOTOR_TO_FLYWHEEL_RATIO = 18/11;
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
