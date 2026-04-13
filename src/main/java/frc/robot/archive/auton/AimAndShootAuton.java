package frc.robot.archive.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.archive.TurretSubsystem;
import frc.robot.archive.vision.LimelightSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.Intake.IntakeSliderSubsystem;

/**
 * Autonomous sequence: aim at an AprilTag, shake the intake pivot while
 * spinning the flywheel and running the feeders, then stop.
 */
public class AimAndShootAuton extends SequentialCommandGroup {

  public AimAndShootAuton(
      TurretSubsystem turret,
      LimelightSubsystem vision,
      FlywheelSubsystem flywheel,
      FeederSubsystem feeder,
      IntakeSliderSubsystem intakePivot) {

    // 1) Compute and set turret target based on latest vision yaw, then wait until within tolerance
    var aimSequence = Commands.sequence(
        Commands.runOnce(() -> {
          if (vision.hasRecentTarget()) {
            double targetDeg = turret.getTurretDeg() + vision.getYawDegMostRecent();
            turret.setTurretAngleDeg(targetDeg);
          }
        }, turret),
        // wait until turret reaches target (within 2 deg) or timeout after 2s
        Commands.waitUntil(() -> Math.abs(turret.getTargetTurretDeg() - turret.getTurretDeg()) < 2.0).withTimeout(2.0)
    );

    // 2) Shoot: run flywheel + feeder and shake intake pivot in parallel for a fixed duration
    var shootParallel = Commands.parallel(
        // Flywheel + feeder runner (sets flywheel power from distance and runs feeder)
        Commands.run(() -> {
          double d = vision.getDistanceMetersMostRecent();
          double power = (d > 0) ? flywheel.getPowerForDistance(d) : 0.6;
          flywheel.setPower(power);
          feeder.feed();
        }, flywheel, feeder).withTimeout(4.0));

        // Intake pivot shake (repeating small up/down pulses) for the same duration
        

    // 3) Stop everything cleanly
    var stopCmd = Commands.runOnce(() -> {
      flywheel.stop();
      feeder.stop();
      intakePivot.stop();
    }, flywheel, feeder, intakePivot);

    addCommands(aimSequence, shootParallel, stopCmd);
  }
}
