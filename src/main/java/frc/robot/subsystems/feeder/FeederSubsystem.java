package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax indexer = new SparkMax(FeederConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax spindexer = new SparkMax(FeederConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

  public FeederSubsystem() {
    SparkMaxConfig idxCfg = new SparkMaxConfig();
    idxCfg.inverted(FeederConstants.INDEXER_INVERTED);
    idxCfg.idleMode(IdleMode.kBrake);
    idxCfg.smartCurrentLimit(FeederConstants.CURRENT_LIMIT);

    SparkMaxConfig spinCfg = new SparkMaxConfig();
    spinCfg.inverted(FeederConstants.SPINDEXER_INVERTED);
    spinCfg.idleMode(IdleMode.kBrake);
    spinCfg.smartCurrentLimit(FeederConstants.CURRENT_LIMIT);

    indexer.configure(idxCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spindexer.configure(spinCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Run both motors forward (ball toward turret). */
  public void feed() {
    indexer.set(FeederConstants.FEED_POWER);
    spindexer.set(FeederConstants.FEED_POWER);
  }

  /** Run both motors backward (unjam / eject). */
  public void reverse() {
    indexer.set(FeederConstants.REVERSE_POWER);
    spindexer.set(FeederConstants.REVERSE_POWER);
  }

  /** Open-loop control if you want it. */
  public void setPower(double indexerPower, double spindexerPower) {
    indexer.set(indexerPower);
    spindexer.set(spindexerPower);
  }

  public void stop() {
    indexer.stopMotor();
    spindexer.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder/IndexerOut", indexer.get());
    SmartDashboard.putNumber("Feeder/SpindexerOut", spindexer.get());
  }
}