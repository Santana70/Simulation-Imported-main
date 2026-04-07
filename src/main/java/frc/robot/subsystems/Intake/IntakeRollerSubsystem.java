// package frc.robot.subsystems.Intake;

// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.RollerConstants;

// public class IntakeRollerSubsystem extends SubsystemBase {

//   private final SparkMax rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

//   public IntakeRollerSubsystem() {
//     SparkMaxConfig cfg = new SparkMaxConfig();
//     cfg.inverted(RollerConstants.ROLLER_INVERTED);
//     cfg.idleMode(IdleMode.kCoast);
//     cfg.smartCurrentLimit(RollerConstants.ROLLER_CURRENT_LIMIT);

//     rollerMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   /** Open-loop percent output (-1 to 1). */
//   public void setPower(double power) {
//     rollerMotor.set(power);
//   }

//   public void intake() {
//     setPower(RollerConstants.INTAKE_POWER);
//   }

//   public void outtake() {
//     setPower(RollerConstants.OUTTAKE_POWER);
//   }

//   public void stop() {
//     rollerMotor.stopMotor();
//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("IntakeRoller/Output", rollerMotor.get());
//   }
// }



package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class IntakeRollerSubsystem extends SubsystemBase {

  // private final TalonFX rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

  // private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final SparkMax rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

  public IntakeRollerSubsystem() {

SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(RollerConstants.ROLLER_INVERTED);
    cfg.idleMode(IdleMode.kCoast);
    cfg.smartCurrentLimit(RollerConstants.ROLLER_CURRENT_LIMIT);

    rollerMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// -------- talon configs
    // TalonFXConfiguration cfg = new TalonFXConfiguration();
    // cfg.MotorOutput = new MotorOutputConfigs()
    //     .withInverted(
    //         RollerConstants.ROLLER_INVERTED
    //             ? InvertedValue.Clockwise_Positive
    //             : InvertedValue.CounterClockwise_Positive)
    //     .withNeutralMode(NeutralModeValue.Coast);

    // cfg.CurrentLimits = new CurrentLimitsConfigs()
    //     .withSupplyCurrentLimit(RollerConstants.ROLLER_CURRENT_LIMIT)
    //     .withSupplyCurrentLimitEnable(true);

    // rollerMotor.getConfigurator().apply(cfg);
  }

  /** Open-loop percent output (-1 to 1). */
  public void setPower(double power) {
    // rollerMotor.setControl(dutyCycleRequest.withOutput(power));  //talon fx
        rollerMotor.set(power);  //sparkmax

  }

  public void intake() {
    setPower(RollerConstants.INTAKE_POWER);
  }

  public void outtake() {
    setPower(RollerConstants.OUTTAKE_POWER);
  }

  public void stop() {
    setPower(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeRoller/Output", rollerMotor.get());
    // SmartDashboard.putNumber("IntakeRoller/StatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("IntakeRoller/SupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
  }
}