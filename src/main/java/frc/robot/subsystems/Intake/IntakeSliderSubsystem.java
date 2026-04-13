package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Controls the intake rack-and-pinion slider.
 *
 * This subsystem supports:
 * - closed-loop position control for preset slider positions
 * - manual open-loop movement for driver control
 * - current-based hard-stop detection
 * - latched direction blocking after hitting a hard stop
 *
 * Latch behavior:
 * - if the slider is moving forward and current stays too high long enough,
 *   forward motion is blocked
 * - forward motion stays blocked until the slider is commanded in reverse
 * - same logic applies in reverse
 */
public class IntakeSliderSubsystem extends SubsystemBase {

  private final SparkMax sliderMotor =
      new SparkMax(IntakeConstants.SLIDER_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder = sliderMotor.getEncoder();
  private final SparkClosedLoopController controller = sliderMotor.getClosedLoopController();

    private final DigitalInput forwardLimitSwitch =
      new DigitalInput(IntakeConstants.SLIDER_FORWARD_LIMIT_DIO);

  private final DigitalInput reverseLimitSwitch =
      new DigitalInput(IntakeConstants.SLIDER_REVERSE_LIMIT_DIO);

  private double goalPosition = IntakeConstants.SLIDER_RETRACTED_POSITION;

  //  1 = forward, -1 = reverse, 0 = stopped
  private int commandedDirection = 0;

  // Latches after a hard stop is detected
  private boolean forwardLimitLatched = false;
  private boolean reverseLimitLatched = false;

  // Current debounce timer start
  private double overCurrentStartTime = -1.0;

  public IntakeSliderSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();

    cfg.inverted(IntakeConstants.SLIDER_INVERTED);
    cfg.idleMode(IdleMode.kCoast);
    cfg.smartCurrentLimit(IntakeConstants.SLIDER_CURRENT_LIMIT);

    // Encoder and setpoints are in slider output units
    cfg.encoder.positionConversionFactor(IntakeConstants.SLIDER_POSITION_PER_MOTOR_ROTATION);
    cfg.encoder.velocityConversionFactor(IntakeConstants.SLIDER_VELOCITY_PER_RPM);

    cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    cfg.closedLoop.p(IntakeConstants.SLIDER_kP);
    cfg.closedLoop.i(IntakeConstants.SLIDER_kI);
    cfg.closedLoop.d(IntakeConstants.SLIDER_kD);
    cfg.closedLoop.outputRange(
        IntakeConstants.SLIDER_MIN_OUTPUT,
        IntakeConstants.SLIDER_MAX_OUTPUT
    );

    sliderMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setPosition(0.0);
    goalPosition = 0.0;
  }

  /** Use when the slider is at a known reference position. */
  public void zeroHere() {
    encoder.setPosition(0.0);
    goalPosition = 0.0;
  }

  /** Manually define the current slider position. */
  public void setCurrentPosition(double currentPosition) {
    encoder.setPosition(currentPosition);
    goalPosition = currentPosition;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getGoalPosition() {
    return goalPosition;
  }

  public double getMotorCurrentAmps() {
    return sliderMotor.getOutputCurrent();
  }

   public boolean isForwardLimitPressed() {
    boolean raw = forwardLimitSwitch.get();
    return IntakeConstants.SLIDER_FORWARD_LIMIT_INVERTED ? !raw : raw;
  }

  public boolean isReverseLimitPressed() {
    boolean raw = reverseLimitSwitch.get();
    return IntakeConstants.SLIDER_REVERSE_LIMIT_INVERTED ? !raw : raw;
  }

  public boolean isForwardLimitLatched() {
    return forwardLimitLatched;
  }

  public boolean isReverseLimitLatched() {
    return reverseLimitLatched;
  }

  private void updateLatchResetFromCommand(double appliedPower) {
    if (appliedPower > 0.0) {
      commandedDirection = 1;
      reverseLimitLatched = false;
    } else if (appliedPower < 0.0) {
      commandedDirection = -1;
      forwardLimitLatched = false;
    } else {
      commandedDirection = 0;
    }
  }

  private void setProtectedPower(double power) {
    double applied = MathUtil.clamp(power, -1.0, 1.0);

    // Hard stop using real limit switches
    if (isForwardLimitPressed() && applied > 0.0) {
      applied = 0.0;
    }

    if (isReverseLimitPressed() && applied < 0.0) {
      applied = 0.0;
    }

    // Backup current-latch protection
    if (forwardLimitLatched && applied > 0.0) {
      applied = 0.0;
    }

    if (reverseLimitLatched && applied < 0.0) {
      applied = 0.0;
    }

    updateLatchResetFromCommand(applied);
    sliderMotor.set(applied);
  }

  public void setGoalPosition(double targetPosition) {
    goalPosition = MathUtil.clamp(
        targetPosition,
        IntakeConstants.SLIDER_MIN_POSITION,
        IntakeConstants.SLIDER_MAX_POSITION
    );

    // Prevent commanding deeper into a pressed hard stop
    if (isForwardLimitPressed() && goalPosition > getPosition()) {
      goalPosition = getPosition();
    }

    if (isReverseLimitPressed() && goalPosition < getPosition()) {
      goalPosition = getPosition();
    }

    controller.setSetpoint(goalPosition, ControlType.kPosition);
  }

  public void retract() {
    setGoalPosition(IntakeConstants.SLIDER_RETRACTED_POSITION);
  }

  public void extend() {
    setGoalPosition(IntakeConstants.SLIDER_EXTENDED_POSITION);
  }

  public void feedPosition() {
    setGoalPosition(IntakeConstants.SLIDER_FEED_POSITION);
  }

  /**
   * Manual control with hard-stop latch protection.
   */
  public void setManualPower(double power) {
    // Freeze the position target at the current location so old position commands
    // do not fight manual control after the operator takes over.
    goalPosition = getPosition();

    double applied = MathUtil.clamp(
        power,
        -IntakeConstants.SLIDER_MANUAL_MAX_POWER,
        IntakeConstants.SLIDER_MANUAL_MAX_POWER
    );

    if (getPosition() <= IntakeConstants.SLIDER_MIN_POSITION && applied < 0) {
      applied = 0.0;
    }
    if (getPosition() >= IntakeConstants.SLIDER_MAX_POSITION && applied > 0) {
      applied = 0.0;
    }

    setProtectedPower(applied);
  }

  public void jogForward() {
    setManualPower(IntakeConstants.SLIDER_MANUAL_POWER);
  }

  public void jogReverse() {
    setManualPower(-IntakeConstants.SLIDER_MANUAL_POWER);
  }

  // Keeps your current bindings from breaking
  public void jogUp() {
    jogForward();
  }

  public void jogDown() {
    jogReverse();
  }

  public boolean atGoal() {
    return Math.abs(goalPosition - getPosition()) <= IntakeConstants.SLIDER_POSITION_TOLERANCE;
  }

  public void stop() {
    commandedDirection = 0;
    sliderMotor.stopMotor();
  }

    /**
   * Changes the slider goal from the current measured position.
   * Positive values extend, negative values retract.
   */
  public void changePositionFromCurrent(double deltaInches) {
    setGoalPosition(getPosition() + deltaInches);
  }

  /** Move slider forward by one preset step. */
  public void stepForward() {
    changePositionFromCurrent(IntakeConstants.SLIDER_STEP_INCHES);
  }

  /** Move slider backward by one preset step. */
  public void stepBackward() {
    changePositionFromCurrent(-IntakeConstants.SLIDER_STEP_INCHES);
  }

  private void updateCurrentLimitLatch() {
    double current = getMotorCurrentAmps();
    boolean overCurrent = current >= IntakeConstants.SLIDER_HARD_STOP_CURRENT_AMPS;

    if (commandedDirection == 0) {
      overCurrentStartTime = -1.0;
      return;
    }

    if (overCurrent) {
      if (overCurrentStartTime < 0.0) {
        overCurrentStartTime = Timer.getFPGATimestamp();
      }

      double overCurrentDuration = Timer.getFPGATimestamp() - overCurrentStartTime;

      if (overCurrentDuration >= IntakeConstants.SLIDER_HARD_STOP_DEBOUNCE_SEC) {
        if (commandedDirection > 0) {
          forwardLimitLatched = true;
        } else if (commandedDirection < 0) {
          reverseLimitLatched = true;
        }

        sliderMotor.stopMotor();
        commandedDirection = 0;
      }
    } else {
      overCurrentStartTime = -1.0;
    }
  }

  @Override
  public void periodic() {
    updateCurrentLimitLatch();

    SmartDashboard.putNumber("IntakeSlider/Position", getPosition());
    SmartDashboard.putNumber("IntakeSlider/Velocity", getVelocity());
    SmartDashboard.putNumber("IntakeSlider/GoalPosition", goalPosition);
    SmartDashboard.putBoolean("IntakeSlider/AtGoal", atGoal());
    SmartDashboard.putNumber("IntakeSlider/AppliedOutput", sliderMotor.get());
    SmartDashboard.putNumber("IntakeSlider/CurrentAmps", getMotorCurrentAmps());
    SmartDashboard.putNumber("IntakeSlider/CommandedDirection", commandedDirection);
    SmartDashboard.putBoolean("IntakeSlider/ForwardLimitLatched", forwardLimitLatched);
    SmartDashboard.putBoolean("IntakeSlider/ReverseLimitLatched", reverseLimitLatched);
  }
}