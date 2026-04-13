package frc.robot.archive.Climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;


public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax climbMotor =
      new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless); // change if brushed

  public ClimberSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(ClimbConstants.CLIMB_INVERTED);
    cfg.idleMode(IdleMode.kBrake);
    cfg.smartCurrentLimit(ClimbConstants.CURRENT_LIMIT);

    climbMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    climbMotor.set(power);
  }

  public void runUp() {
    setPower(ClimbConstants.UP_POWER);
  }

  public void runDown() {
    setPower(ClimbConstants.DOWN_POWER);
  }

  public void stop() {
    climbMotor.stopMotor();
  }

  public Command runForTime(double seconds, double power) {
    return Commands.startEnd(
        () -> setPower(power),
        this::stop,
        this
    ).withTimeout(seconds);
  }

  public Command runUpForTime(double seconds) {
    return runForTime(seconds, ClimbConstants.UP_POWER);
  }

  public Command runDownForTime(double seconds) {
    return runForTime(seconds, ClimbConstants.DOWN_POWER);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb/Output", climbMotor.get());
  }
}