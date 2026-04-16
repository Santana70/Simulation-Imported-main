package frc.robot.archive;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RollerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Flywheel extends SubsystemBase {

  // private final SparkMax flywheel = new SparkMax(FlywheelConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
  private final TalonFX flywheel = new TalonFX(FlywheelConstants.FLYWHEEL_MOTOR_ID);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  public Flywheel() {
    // SparkMaxConfig cfg = new SparkMaxConfig();
    // cfg.inverted(FlywheelConstants.FLYWHEEL_INVERTED);
    // cfg.idleMode(IdleMode.kCoast);
    // cfg.smartCurrentLimit(FlywheelConstants.CURRENT_LIMIT);

    // Configure TalonFX instead of SparkMax
      TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput = new MotorOutputConfigs()
        .withInverted(
            FlywheelConstants.FLYWHEEL_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

    cfg.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(RollerConstants.ROLLER_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);

    flywheel.getConfigurator().apply(cfg);

    SmartDashboard.setDefaultNumber("LL/ShotD1", 1.5);
SmartDashboard.setDefaultNumber("LL/ShotP1", 0.57);
SmartDashboard.setDefaultNumber("LL/ShotD2", 1.8);
SmartDashboard.setDefaultNumber("LL/ShotP2", 0.55);
SmartDashboard.setDefaultNumber("LL/ShotD3", 2.1);
SmartDashboard.setDefaultNumber("LL/ShotP3", 0.56);
SmartDashboard.setDefaultNumber("LL/ShotD4", 2.4);
SmartDashboard.setDefaultNumber("LL/ShotP4", 0.63);
SmartDashboard.setDefaultNumber("LL/ShotD5", 2.7);
SmartDashboard.setDefaultNumber("LL/ShotP5", 0.65);
SmartDashboard.setDefaultNumber("LL/ShotD6", 3.0);
SmartDashboard.setDefaultNumber("LL/ShotP6", 0.70);
SmartDashboard.setDefaultNumber("LL/ShotD7", 3.3);
SmartDashboard.setDefaultNumber("LL/ShotP7", 0.80);
SmartDashboard.setDefaultNumber("LL/ShotPDefault", 0.20);

SmartDashboard.setDefaultNumber("POSE/ShotD1", 1.5);
SmartDashboard.setDefaultNumber("POSE/ShotP1", 0.);
SmartDashboard.setDefaultNumber("POSE/ShotD2", 1.8);
SmartDashboard.setDefaultNumber("POSE/ShotP2", 0.);
SmartDashboard.setDefaultNumber("POSE/ShotD3", 2.1);
SmartDashboard.setDefaultNumber("POSE/ShotP3", 0.5);
SmartDashboard.setDefaultNumber("POSE/ShotD4", 2.4);
SmartDashboard.setDefaultNumber("POSE/ShotP4", 0.52);
SmartDashboard.setDefaultNumber("POSE/ShotD5", 2.7);
SmartDashboard.setDefaultNumber("POSE/ShotP5", 0.58);
SmartDashboard.setDefaultNumber("POSE/ShotD6", 3.0);
SmartDashboard.setDefaultNumber("POSE/ShotP6", 0.6);
SmartDashboard.setDefaultNumber("POSE/ShotD7", 3.5);
SmartDashboard.setDefaultNumber("POSE/ShotP7", 0.63);
SmartDashboard.setDefaultNumber("POSE/ShotD8", 4.0);
SmartDashboard.setDefaultNumber("POSE/ShotP8", 0.64);
SmartDashboard.setDefaultNumber("POSE/ShotD9", 4.5);
SmartDashboard.setDefaultNumber("POSE/ShotP9", 0.64);
SmartDashboard.setDefaultNumber("POSE/ShotD10", 5.0);
SmartDashboard.setDefaultNumber("POSE/ShotP10", 0.65);
SmartDashboard.setDefaultNumber("POSE/ShotD11", 5.6);
SmartDashboard.setDefaultNumber("POSE/ShotP11", 0.67);

SmartDashboard.setDefaultNumber("POSE/ShotPDefault", 0.75);
  }

  /** Open-loop percent output (-1 to 1). */
  public void setPower(double power) {
    flywheel.set(power);
  }

  /** Full send open-loop (change constant to something lower if needed). */
  public void shoot() {
    setPower(FlywheelConstants.SHOOT_POWER);
  }

  public void stop() {
    setPower(FlywheelConstants.IDLE_POWER);
    flywheel.stopMotor();
  }

  private static final Translation2d HUB_BLUE = new Translation2d(4.6, 4.0);

public double getPowerForPose(Pose2d robotPose) {
  double distanceMeters = robotPose.getTranslation().getDistance(HUB_BLUE);
  SmartDashboard.putNumber("Flywheel/PoseDistanceMeters", distanceMeters);
  return getPowerForDistance(distanceMeters);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel/Output", flywheel.get());
  }

  public double getPowerForDistance(double distanceMeters) {

  if (distanceMeters < 1.5) return 0.57; //about .5
  if (distanceMeters < 1.8) return 0.55; //about .5
  if (distanceMeters < 2.1) return 0.56; //about .5
  if (distanceMeters < 2.4) return 0.63; //about .5
  if (distanceMeters < 2.7) return 0.65; //about .5
  if (distanceMeters < 3) return 0.7; //about .7
  if (distanceMeters < 3.3) return 0.8; //about .8
  return 0.6;
}
public double getPowerForLLDistance(double distanceMeters) {
  if (distanceMeters < SmartDashboard.getNumber("LL/ShotD1", 1.5)) {
    return SmartDashboard.getNumber("LL/ShotP1", 0.57);
  }
  if (distanceMeters < SmartDashboard.getNumber("LL/ShotD2", 1.8)) {
    return SmartDashboard.getNumber("LL/ShotP2", 0.55);
  }
  if (distanceMeters < SmartDashboard.getNumber("LL/ShotD3", 2.1)) {
    return SmartDashboard.getNumber("LL/ShotP3", 0.56);
  }
  if (distanceMeters < SmartDashboard.getNumber("LL/ShotD4", 2.4)) {
    return SmartDashboard.getNumber("LL/ShotP4", 0.63);
  }
  if (distanceMeters < SmartDashboard.getNumber("LL/ShotD5", 2.7)) {
    return SmartDashboard.getNumber("LL/ShotP5", 0.65);
  }
  if (distanceMeters < SmartDashboard.getNumber("LL/ShotD6", 3.0)) {
    return SmartDashboard.getNumber("LL/ShotP6", 0.70);
  }
  if (distanceMeters < SmartDashboard.getNumber("LL/ShotD7", 3.3)) {
    return SmartDashboard.getNumber("LL/ShotP7", 0.80);
  }
  return SmartDashboard.getNumber("LL/ShotPDefault", 0.20);
}

public double getPowerForPoseDistance(double distanceMeters) {
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD1", 1.5)) {
    return SmartDashboard.getNumber("POSE/ShotP1", .53);
  }
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD2", 1.8)) {
    return SmartDashboard.getNumber("POSE/ShotP2", 0.54);
  }
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD3", 2.1)) {
    return SmartDashboard.getNumber("POSE/ShotP3", 0.55);
  }
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD4", 2.4)) {
    return SmartDashboard.getNumber("POSE/ShotP4", 0.56);
  }
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD5", 2.7)) {
    return SmartDashboard.getNumber("POSE/ShotP5", 0.58);
  }
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD6", 3.0)) {
    return SmartDashboard.getNumber("POSE/ShotP6", 0.60);
  }
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD7", 3.5)) {
    return SmartDashboard.getNumber("POSE/ShotP7", 0.63);
  }
  if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD8", 4.0)) {
    return SmartDashboard.getNumber("POSE/ShotP8", 0.64);
  }
    if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD9", 4.5)) {
    return SmartDashboard.getNumber("POSE/ShotP9", 0.64);
  } 
   if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD10", 5.0)) {  
    return SmartDashboard.getNumber("POSE/ShotP10", .65);
  }
    if (distanceMeters < SmartDashboard.getNumber("POSE/ShotD11", 5.6)) {
    return SmartDashboard.getNumber("POSE/ShotP11", .67);
  }
  return SmartDashboard.getNumber("POSE/ShotPDefault", 0.70);
}





}