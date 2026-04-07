package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;



  public class IntakeSliderSubsystem extends SubsystemBase {

  private final SparkMax pivotMotor =
      new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = pivotMotor.getEncoder();
  private final SparkClosedLoopController controller = pivotMotor.getClosedLoopController();

  private double goalDeg = IntakeConstants.PIVOT_STOW_ANGLE_DEG;

  public IntakeSliderSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();

    cfg.inverted(IntakeConstants.PIVOT_INVERTED);
    cfg.idleMode(IdleMode.kBrake);
    cfg.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT);

    // Make encoder + setpoints read in pivot degrees
    cfg.encoder.positionConversionFactor(IntakeConstants.PIVOT_DEG_PER_MOTOR_ROT);
    cfg.encoder.velocityConversionFactor(IntakeConstants.PIVOT_DEG_PER_SEC_PER_RPM);

    cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    cfg.closedLoop.p(IntakeConstants.PIVOT_kP);
    cfg.closedLoop.i(IntakeConstants.PIVOT_kI);
    cfg.closedLoop.d(IntakeConstants.PIVOT_kD);
    cfg.closedLoop.outputRange(
        IntakeConstants.PIVOT_MIN_OUTPUT,
        IntakeConstants.PIVOT_MAX_OUTPUT
    );

    pivotMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Since this is only a relative encoder, startup zero is arbitrary.
    // You manually call zeroHere() when the mechanism is at known zero.
    encoder.setPosition(0.0);
    goalDeg = 0.0;
  }

  /** Manual zero when pivot is physically at your known zero angle. */
  public void zeroHere() {
    encoder.setPosition(0.0);
    goalDeg = 0.0;
  }

  /** Optional: zero to some known angle other than 0. */
  public void setCurrentAngleDeg(double currentAngleDeg) {
    encoder.setPosition(currentAngleDeg);
    goalDeg = currentAngleDeg;
  }

  public double getAngleDeg() {
    return encoder.getPosition();
  }

  public double getVelocityDegPerSec() {
    return encoder.getVelocity();
  }

  public double getGoalDeg() {
    return goalDeg;
  }

  public void setGoalDeg(double targetDeg) {
    goalDeg = MathUtil.clamp(
        targetDeg,
        IntakeConstants.PIVOT_MIN_ANGLE_DEG,
        IntakeConstants.PIVOT_MAX_ANGLE_DEG
    );

    controller.setSetpoint(goalDeg, ControlType.kPosition);
  }

  public void stow() {
    setGoalDeg(IntakeConstants.PIVOT_STOW_ANGLE_DEG);
  }

  public void deploy() {
    setGoalDeg(IntakeConstants.PIVOT_DEPLOY_ANGLE_DEG);
  }

  public void feedPosition() {
    setGoalDeg(IntakeConstants.PIVOT_FEED_ANGLE_DEG);
  }

  /** Manual open-loop jog for tuning or emergency control. */
  public void setManualPower(double power) {
    double applied = MathUtil.clamp(power, -0.4, 0.4);

    // Soft-limit protection in manual mode
    if (getAngleDeg() <= IntakeConstants.PIVOT_MIN_ANGLE_DEG && applied < 0) {
      applied = 0.0;
    }
    if (getAngleDeg() >= IntakeConstants.PIVOT_MAX_ANGLE_DEG && applied > 0) {
      applied = 0.0;
    }

    pivotMotor.set(applied);
  }

  public void jogUp() {
    setManualPower(IntakeConstants.PIVOT_MANUAL_POWER);
  }

  public void jogDown() {
    setManualPower(-IntakeConstants.PIVOT_MANUAL_POWER);
  }

  public boolean atGoal() {
    return Math.abs(goalDeg - getAngleDeg()) <= IntakeConstants.PIVOT_TOLERANCE_DEG;
  }

  public void stop() {
    pivotMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakePivot/AngleDeg", getAngleDeg());
    SmartDashboard.putNumber("IntakePivot/VelocityDegPerSec", getVelocityDegPerSec());
    SmartDashboard.putNumber("IntakePivot/GoalDeg", goalDeg);
    SmartDashboard.putBoolean("IntakePivot/AtGoal", atGoal());
    SmartDashboard.putNumber("IntakePivot/AppliedOutput", pivotMotor.get());
  }
}