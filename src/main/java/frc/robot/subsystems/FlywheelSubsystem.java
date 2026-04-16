package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

   private static final int FLYWHEEL_MOTOR_ID = Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID;
private static final boolean MOTOR_INVERTED = Constants.FlywheelConstants.FLYWHEEL_INVERTED;
private static final double MOTOR_TO_FLYWHEEL_RATIO = Constants.FlywheelConstants.MOTOR_TO_FLYWHEEL_RATIO;
private static final double TARGET_RPM = Constants.FlywheelConstants.NORMAL_SHOT_RPM;
private static final double RPM_TOLERANCE = Constants.FlywheelConstants.RPM_TOLERANCE;
private static final double SUPPLY_CURRENT_LIMIT = Constants.FlywheelConstants.SUPPLY_CURRENT_LIMIT;
private static final double STATOR_CURRENT_LIMIT = Constants.FlywheelConstants.STATOR_CURRENT_LIMIT;

private static final double kS = Constants.FlywheelConstants.kS;
private static final double kV = Constants.FlywheelConstants.kV;
private static final double kA = Constants.FlywheelConstants.kA;
private static final double kP = Constants.FlywheelConstants.kP;
private static final double kI = Constants.FlywheelConstants.kI;
private static final double kD = Constants.FlywheelConstants.kD;
    // =========================
    // OPTIONAL RAMP / FILTERING
    // =========================

    // Smoothing for measured acceleration so feedforward is less noisy
    private static final int ACCEL_FILTER_TAPS = 8;

    private final TalonFX flywheelMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final CoastOut coastRequest = new CoastOut();

    private final LinearFilter accelFilter = LinearFilter.movingAverage(ACCEL_FILTER_TAPS);

    private double targetRPM = 0.0;
    private double lastMeasuredRPS = 0.0;
    private double filteredAccelRPSPerSec = 0.0;
    private double lastTimestampSec = 0.0;

    public FlywheelSubsystem() {
        flywheelMotor = new TalonFX(FLYWHEEL_MOTOR_ID);

        configureMotor();

        lastTimestampSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive);

        config.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(MOTOR_TO_FLYWHEEL_RATIO);

        config.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        config.Slot0 = new Slot0Configs()
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKP(kP)
                .withKI(kI)
                .withKD(kD);

        flywheelMotor.getConfigurator().apply(config);

        // Voltage compensation behavior in Phoenix 6 is handled by voltage-based control requests,
        // so VelocityVoltage is already the correct control mode for consistency.
    }

    /**
     * Spins the flywheel to the default target RPM.
     */
    public void runFlywheel() {
        setTargetRPM(TARGET_RPM);
    }

    /**
     * Spins the flywheel to a custom RPM.
     */
    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0.0, rpm);

        double targetRPS = rpm / 60.0;

        flywheelMotor.setControl(
                velocityRequest.withVelocity(targetRPS)
        );
    }

    /**
     * Stops the flywheel.
     */
    public void stopFlywheel() {
        targetRPM = 0.0;
        flywheelMotor.setControl(neutralRequest);
    }

    /**
     * Lets the flywheel coast down naturally.
     */
    public void coastFlywheel() {
        targetRPM = 0.0;
        flywheelMotor.setControl(coastRequest);
    }

    /**
     * Use only for testing.
     */
    public void setPercentOutput(double percent) {
        targetRPM = 0.0;
        flywheelMotor.set(percent);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getCurrentRPS() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    public double getMotorVoltage() {
        return flywheelMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return flywheelMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return flywheelMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getClosedLoopErrorRPM() {
        return (targetRPM - getCurrentRPM());
    }

    public boolean atSpeed() {
        return Math.abs(getClosedLoopErrorRPM()) <= RPM_TOLERANCE;
    }

    public boolean isSpinning() {
        return getCurrentRPM() > 200.0;
    }

    public void setPower(double power) {
    targetRPM = 0.0;
    flywheelMotor.set(power);
}

public void stop() {
    stopFlywheel();
}

public void shoot() {
    setPercentOutput(0.97);
}

public double getPowerForDistance(double distanceMeters) {
    if (distanceMeters < 1.5) return 0.57;
    if (distanceMeters < 1.8) return 0.55;
    if (distanceMeters < 2.1) return 0.56;
    if (distanceMeters < 2.4) return 0.63;
    if (distanceMeters < 2.7) return 0.65;
    if (distanceMeters < 3.0) return 0.70;
    if (distanceMeters < 3.3) return 0.80;
    return 0.60;
}

public double getPowerForPoseDistance(double distanceMeters) {
    if (distanceMeters < 1.5) return 0.53;
    if (distanceMeters < 1.8) return 0.54;
    if (distanceMeters < 2.1) return 0.55;
    if (distanceMeters < 2.4) return 0.56;
    if (distanceMeters < 2.7) return 0.58;
    if (distanceMeters < 3.0) return 0.60;
    if (distanceMeters < 3.5) return 0.63;
    if (distanceMeters < 4.0) return 0.64;
    if (distanceMeters < 4.5) return 0.64;
    if (distanceMeters < 5.0) return 0.65;
    if (distanceMeters < 5.6) return 0.67;
    return 0.70;
}

    @Override
    public void periodic() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double dt = now - lastTimestampSec;

        double measuredRPS = getCurrentRPS();

        if (dt > 1e-4) {
            double rawAccel = (measuredRPS - lastMeasuredRPS) / dt;
            filteredAccelRPSPerSec = accelFilter.calculate(rawAccel);
        }

        lastMeasuredRPS = measuredRPS;
        lastTimestampSec = now;

        SmartDashboard.putNumber("Flywheel/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/CurrentRPM", getCurrentRPM());
        SmartDashboard.putNumber("Flywheel/ErrorRPM", getClosedLoopErrorRPM());
        SmartDashboard.putBoolean("Flywheel/AtSpeed", atSpeed());
        SmartDashboard.putNumber("Flywheel/MotorVoltage", getMotorVoltage());
        SmartDashboard.putNumber("Flywheel/SupplyCurrent", getSupplyCurrent());
        SmartDashboard.putNumber("Flywheel/StatorCurrent", getStatorCurrent());
        SmartDashboard.putNumber("Flywheel/FilteredAccelRPSPerSec", filteredAccelRPSPerSec);
    }
}