package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

  private static final int TURRET_CAN_ID = 25;
  private static final int MAG12_TALON_ID = 30;
  private static final int MAG13_TALON_ID = 31;

  private static final MotorType MOTOR_TYPE = MotorType.kBrushed;

  private static final double GEAR_RATIO_MOTOR_PER_TURRET = 77.4;

  private static final double MIN_TURRET_DEG = -200.0;
  private static final double MAX_TURRET_DEG = 180.0;

  private static final double MAX_OPEN_LOOP_OUTPUT = 0.90;
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double DEG_PER_MOTOR_ROT = 360.0 / GEAR_RATIO_MOTOR_PER_TURRET;

  private static final double TURRET_TEETH = 86.0;
  private static final double ENC12_TEETH = 12.0;
  private static final double ENC13_TEETH = 13.0;

  private static final double ENC12_ROT_PER_TURRET = TURRET_TEETH / ENC12_TEETH;
  private static final double ENC13_ROT_PER_TURRET = TURRET_TEETH / ENC13_TEETH;

  private static final int SRX_MASK = 0xFFF;
  private static final double SRX_CPR = 4096.0;

  private static final double MIN_TURRET_ROT = MIN_TURRET_DEG / 360.0;
  private static final double MAX_TURRET_ROT = MAX_TURRET_DEG / 360.0;
  private static final double DEG_PER_ENCODER_UNIT = 740.0;

  private static final Angle MATCH_TOL = Units.Rotations.of(0.06);

  private final SparkMax motor = new SparkMax(TURRET_CAN_ID, MOTOR_TYPE);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();
  private final RelativeEncoder encoder;

  private final WPI_TalonSRX mag12Host = new WPI_TalonSRX(MAG12_TALON_ID);
  private final WPI_TalonSRX mag13Host = new WPI_TalonSRX(MAG13_TALON_ID);

  private final EasyCRT easyCrt;

  private double targetTurretDeg = 0.0;
  private boolean closedLoopEnabled = false;
  private boolean encoderInDegrees = false;

  public TurretSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();

    cfg.idleMode(IdleMode.kBrake)
       .smartCurrentLimit(30)
       .voltageCompensation(12.0);

    cfg.alternateEncoder
        .setSparkMaxDataPortConfig()
        .countsPerRevolution(28);

    cfg.closedLoop
        .p(kP)
        .i(kI)
        .d(kD);

    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = motor.getEncoder();

    try {
      java.lang.reflect.Method m = encoder.getClass().getMethod("setPositionConversionFactor", double.class);
      m.invoke(encoder, DEG_PER_MOTOR_ROT);
      encoderInDegrees = true;
    } catch (Exception ignored) {
      encoderInDegrees = false;
    }

    Supplier<Angle> abs12 = () -> Units.Rotations.of(readSrxMagAbs01(mag12Host));
    Supplier<Angle> abs13 = () -> Units.Rotations.of(readSrxMagAbs01(mag13Host));

    EasyCRTConfig crtCfg =
        new EasyCRTConfig(abs12, abs13)
            .withEncoderRatios(ENC12_ROT_PER_TURRET, ENC13_ROT_PER_TURRET)
            .withMechanismRange(Units.Rotations.of(MIN_TURRET_ROT), Units.Rotations.of(MAX_TURRET_ROT))
            .withMatchTolerance(MATCH_TOL)
            .withAbsoluteEncoderOffsets(Units.Rotations.of(0.0), Units.Rotations.of(0.0))
            .withAbsoluteEncoderInversions(false, false);

    easyCrt = new EasyCRT(crtCfg);

    encoder.setPosition(0.0);
    targetTurretDeg = 0.0;
  }
public double getTurretDeg() {
  return encoder.getPosition() * DEG_PER_ENCODER_UNIT;
}

public double getRawPosition() {
  return encoder.getPosition();
}

  public double getTargetTurretDeg() {
    return targetTurretDeg;
  }

  public double getTurretVelocityDegPerSec() {
    double rpm = encoder.getVelocity();
    double degPerMin = rpm * DEG_PER_MOTOR_ROT;
    return degPerMin / 60.0;
  }

  public void zeroTurretHere() {
    encoder.setPosition(0.0);
    targetTurretDeg = 0.0;
  }

  public void seedFromCrt() {
    Optional<Angle> opt = easyCrt.getAngleOptional();

    if (opt.isPresent()) {
      double turretRot = opt.get().in(Units.Rotations);
      double turretDeg = turretRot * 360.0;

      if (encoderInDegrees) {
        encoder.setPosition(turretDeg);
      } else {
        encoder.setPosition(turretDeg / DEG_PER_MOTOR_ROT);
      }

      targetTurretDeg = turretDeg;

      DriverStation.reportWarning(
          String.format("Turret CRT OK: turretDeg=%.2f status=%s",
              turretDeg, easyCrt.getLastStatus()),
          false);
    } else {
      DriverStation.reportError(
          "Turret CRT FAILED: status=" + easyCrt.getLastStatus(),
          false);
    }
  }

  public void setOpenLoop(double percent) {
    closedLoopEnabled = false;

    double turretDeg = getTurretDeg();
    if (turretDeg <= MIN_TURRET_DEG && percent < 0) percent = 0.0;
    if (turretDeg >= MAX_TURRET_DEG && percent > 0) percent = 0.0;

    percent = MathUtil.clamp(percent, -MAX_OPEN_LOOP_OUTPUT, MAX_OPEN_LOOP_OUTPUT);
    motor.set(percent);
  }

  public void stop() {
    closedLoopEnabled = false;
    motor.set(0.0);
  }

  public void setRawMotorPosition(double rawSetpoint) {
    closedLoopEnabled = true;
    controller.setReference(rawSetpoint, ControlType.kPosition);
  }

  public void aimAtFieldPoint(double targetX, double targetY, edu.wpi.first.math.geometry.Pose2d robotPose) {
  double dx = targetX - robotPose.getX();
  double dy = targetY - robotPose.getY();

  double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
  double robotHeadingDeg = robotPose.getRotation().getDegrees();

  double turretTargetDeg = fieldAngleDeg - robotHeadingDeg + 180;

  // wrap to [-180, 180]
  turretTargetDeg = MathUtil.inputModulus(turretTargetDeg, -180.0, 180.0);

  setTurretAngleDeg(turretTargetDeg);
}

// public void setTurretAngleDeg(double turretDeg) {
//   closedLoopEnabled = true;
//   targetTurretDeg = MathUtil.clamp(turretDeg, MIN_TURRET_DEG, MAX_TURRET_DEG);

//   double setpointNative = targetTurretDeg / DEG_PER_ENCODER_UNIT;
//   controller.setReference(setpointNative, ControlType.kPosition);
// }
public void setTurretAngleDeg(double turretDeg) {
  targetTurretDeg = MathUtil.clamp(turretDeg, MIN_TURRET_DEG, MAX_TURRET_DEG);
  closedLoopEnabled = true;
}

public boolean atTarget() {
  return Math.abs(targetTurretDeg - getTurretDeg()) < 2.0;
}

  private static double readSrxMagAbs01(WPI_TalonSRX hostTalon) {
    int pw = hostTalon.getSensorCollection().getPulseWidthPosition() & SRX_MASK;
    return pw / SRX_CPR;
  }

  @Override
  public void periodic() {

    if (closedLoopEnabled) {
    double error = targetTurretDeg - getTurretDeg();

    if (Math.abs(error) < 2.0) {
      motor.set(0.0);
    } else {
      double out = 0.05 * error;

      // minimum output to overcome friction
      out = Math.copySign(Math.max(Math.abs(out), 0.43), out);

      // clamp
      out = MathUtil.clamp(out, -0.9, 0.9227);

      motor.set(out);
    }
  }
  

    SmartDashboard.putNumber("Turret/AngleDeg", getTurretDeg());
    SmartDashboard.putNumber("Turret/TargetDeg", targetTurretDeg);
    SmartDashboard.putNumber("Turret/ErrorDeg", targetTurretDeg - getTurretDeg());
    SmartDashboard.putNumber("Turret/EncoderPosRaw", encoder.getPosition());
    SmartDashboard.putNumber("Turret/Velocity", getTurretVelocityDegPerSec());
    SmartDashboard.putBoolean("Turret/ClosedLoop", closedLoopEnabled);

    SmartDashboard.putString("Turret/CRTStatus", String.valueOf(easyCrt.getLastStatus()));
    SmartDashboard.putNumber("Turret/Abs12", readSrxMagAbs01(mag12Host));
    SmartDashboard.putNumber("Turret/Abs13", readSrxMagAbs01(mag13Host));
  }
}