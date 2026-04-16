package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.Intake.IntakeRollerSubsystem;
import frc.robot.subsystems.Intake.IntakeSliderSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * Stores reusable robot actions so RobotContainer stays clean.
 *
 * RobotContainer should focus on:
 * - creating subsystems
 * - controller bindings
 * - autonomous chooser
 *
 * This class focuses on:
 * - small reusable commands
 * - grouped mechanism actions
 * - commands used in both teleop and autonomous
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

  /** Turns intake rollers on. */
  public Command intakeOnCommand() {
    return Commands.runOnce(intakeRoller::intake, intakeRoller);
  }

  /** Turns intake rollers off. */
  public Command intakeOffCommand() {
    return Commands.runOnce(intakeRoller::stop, intakeRoller);
  }

/** Retracts the intake slider, waits briefly, then stops motor output. */
public Command stowIntakeCommand() {
  return Commands.sequence(
      Commands.run(intakeSlider::retract, intakeSlider).withTimeout(0.6),
      Commands.runOnce(intakeSlider::stop, intakeSlider));
}

/** Extends the intake slider, waits briefly, then stops motor output. */
public Command deployIntakeCommand() {
  return Commands.sequence(
      Commands.run(intakeSlider::extend, intakeSlider).withTimeout(0.6),
      Commands.runOnce(intakeSlider::stop, intakeSlider));
}
  /**
   * Shoots using a supplied distance in meters.
   *
   * Flow:
   * 1. Compute flywheel power from distance
   * 2. Spin flywheel immediately
   * 3. Wait for flywheel spin-up
   * 4. Start feeder
   */
  // public Command shootFromDistanceCommand(DoubleSupplier distanceSupplier) {
  //   return Commands.run(() -> {
  //         double distanceMeters = distanceSupplier.getAsDouble();
  //         SmartDashboard.putNumber("Shot/DistanceMeters", distanceMeters);

  //         double flywheelPower = flywheel.getPowerForPoseDistance(distanceMeters);
  //         SmartDashboard.putNumber("Shot/CommandedPower", flywheelPower);

  //         flywheel.setPower(flywheelPower);
  //       }, flywheel)
  //       .alongWith(
  //           Commands.sequence(
  //               Commands.waitSeconds(1.5),
  //               Commands.run(feeder::feed, feeder)
  //           )
  //       );
  // }

  public Command shootRPMCommand(double rpm) {
  return Commands.run(() -> flywheel.setTargetRPM(rpm), flywheel)
      .alongWith(
          Commands.sequence(
              Commands.waitSeconds(1.5),
              Commands.run(feeder::feed, feeder)
          )
      );
}

  /** Fixed-power shot for simple testing. */
  public Command fixedShotCommand(double flywheelPower) {
    return Commands.run(() -> flywheel.setPower(flywheelPower), flywheel)
        .alongWith(
            Commands.sequence(
                Commands.waitSeconds(1.5),
                Commands.run(feeder::feed, feeder)
            )
        );
  }

  /** Stops shooter-related mechanisms. */
  public Command stopShooterCommand() {
    return Commands.runOnce(() -> {
      feeder.stop();
      flywheel.stop();
    }, feeder, flywheel);
  }

  /** Stops all intake/shooter mechanisms. */
  public Command panicStopCommand() {
    return Commands.runOnce(() -> {
      intakeSlider.stop();
      intakeRoller.stop();
      feeder.stop();
      flywheel.stop();
    }, 
    intakeSlider, 
    intakeRoller, feeder, flywheel);
  }
}