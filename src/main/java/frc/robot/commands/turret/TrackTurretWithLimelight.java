package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretAimConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;

public class TrackTurretWithLimelight extends Command {

  private final TurretSubsystem turret;
  private final LimelightSubsystem vision;

  public TrackTurretWithLimelight(TurretSubsystem turret2, LimelightSubsystem vision) {
    this.turret = turret2;
    this.vision = vision;
    addRequirements(turret2);
  }

  @Override
  public void execute() {
    if (!vision.hasTarget()) {
      turret.stop();
      return;
    }

    double tx = vision.getYawDeg(); // degrees
    if (Math.abs(tx) < TurretAimConstants.AIM_DEADBAND_DEG) {
      turret.stop();
      return;
    }

    double out = TurretAimConstants.kP * tx;

    // add a minimum output to overcome static friction
    out = Math.copySign(Math.max(Math.abs(out), TurretAimConstants.MIN_OUT), out);

    // clamp output
    out = MathUtil.clamp(out, -TurretAimConstants.MAX_OUT, TurretAimConstants.MAX_OUT);

    // If it aims the wrong way, flip the sign: out = -out;
    turret.setOpenLoop(-out);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}