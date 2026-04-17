package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.Intake.IntakeRollerSubsystem;
import frc.robot.subsystems.Intake.IntakeSliderSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * Stores reusable mechanism commands so RobotContainer stays cleaner.
 *
 * Put grouped actions here:
 * - intake actions
 * - shooter actions
 * - autonomous helper actions
 *
 * Put button bindings in RobotContainer.
 */
public class RobotCommands {

  private final IntakeSliderSubsystem intakeSlider;
  private final IntakeRollerSubsystem intakeRoller;
  private final FeederSubsystem feeder;
  private final FlywheelSubsystem flywheel;

  public RobotCommands(
      IntakeSliderSubsystem intakeSlider,
      IntakeRollerSubsystem intakeRoller,
      FeederSubsystem feeder,
      FlywheelSubsystem flywheel) {
    this.intakeSlider = intakeSlider;
    this.intakeRoller = intakeRoller;
    this.feeder = feeder;
    this.flywheel = flywheel;
  }

  /** Turns intake roller on to pull game pieces in. */
  public Command intakeOnCommand() {
    return Commands.runOnce(intakeRoller::intake, intakeRoller);
  }

  /** Stops the intake roller. */
  public Command intakeOffCommand() {
    return Commands.runOnce(intakeRoller::stop, intakeRoller);
  }

  /**
   * Moves the intake slider to the fully retracted preset.
   * Waits until it arrives or times out.
   */
  public Command stowIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(intakeSlider::retract, intakeSlider),
        Commands.waitUntil(intakeSlider::atGoal).withTimeout(1.0)
    );
  }

  /**
   * Moves the intake slider to the fully extended preset.
   * Waits until it arrives or times out.
   */
  public Command deployIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(intakeSlider::extend, intakeSlider),
        Commands.waitUntil(intakeSlider::atGoal).withTimeout(1.0)
    );
  }

  /**
   * Moves the intake slider to the feed position preset.
   * This is useful in auto before feeding/shooting.
   */
  public Command feedIntakePositionCommand() {
    return Commands.sequence(
        Commands.runOnce(intakeSlider::feedPosition, intakeSlider),
        Commands.waitUntil(intakeSlider::atGoal).withTimeout(1.0)
    );
  }

  /**
   * Teleop/held RPM shooting command.
   * While this command is running:
   * - flywheel is held at target RPM
   * - feeder starts after a short spin-up delay
   *
   * Best for buttons that are held, not one-tap auto markers.
   */
  public Command shootRPMCommand(double rpm) {
    return Commands.run(() -> flywheel.setTargetRPM(rpm), flywheel)
        .alongWith(
            Commands.sequence(
                Commands.waitSeconds(1.5),
                Commands.run(feeder::feed, feeder)
            )
        );
  }

  /**
   * Simple fixed power backup shot.
   * Use only if RPM control is not working yet.
   */
  public Command fixedShotCommand(double flywheelPower) {
    return Commands.run(() -> flywheel.setPower(flywheelPower), flywheel)
        .alongWith(
            Commands.sequence(
                Commands.waitSeconds(1.5),
                Commands.run(feeder::feed, feeder)
            )
        );
  }

  /**
   * Autonomous RPM shot.
   *
   * Flow:
   * 1. Spin flywheel to target RPM
   * 2. Wait for spin-up
   * 3. Feed for a fixed time
   * 4. Stop feeder and flywheel
   *
   * This is better for PathPlanner event markers than a forever-running command.
   */
public Command autoShotRPM(double rpm) {
  return Commands.sequence(
      Commands.runOnce(() -> flywheel.setTargetRPM(rpm), flywheel),
      Commands.waitSeconds(1.5),
      Commands.run(feeder::feed, feeder).withTimeout(5.0),
      Commands.runOnce(() -> {
        feeder.stop();
        flywheel.stopFlywheel();
      }, feeder, flywheel)
  );
}

  /** Stops feeder and flywheel only. */
  public Command stopShooterCommand() {
    return Commands.runOnce(() -> {
      feeder.stop();
      flywheel.stop();
    }, feeder, flywheel);
  }

  /** Emergency stop for all major mechanisms. */
  public Command panicStopCommand() {
    return Commands.runOnce(() -> {
      intakeSlider.stop();
      intakeRoller.stop();
      feeder.stop();
      flywheel.stop();
    }, intakeSlider, intakeRoller, feeder, flywheel);
  }
}