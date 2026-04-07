package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.turret.TrackTurretWithLimelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.Intake.IntakeSliderSubsystem;
import frc.robot.subsystems.Intake.IntakeRollerSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.turret.FlywheelSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.Climb.ClimberSubsystem;
import frc.robot.auton.AimAndShootAuton;
import com.pathplanner.lib.auto.NamedCommands;

import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.SwerveInputStream;
import yams.mechanisms.positional.Pivot;

public class RobotContainer {
  private static final double START_HEADING_DEG = 0.0;
  private static final double BACK_HEADING_DEG = 0.0;
  private static final double CENTER_HEADING_DEG = 0.0;
  // private final ClimberSubsystem climb = new ClimberSubsystem();
  private final IntakeSliderSubsystem intakeSlider = new IntakeSliderSubsystem();
  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  // public final TurretSubsystem turret = new TurretSubsystem();
  private double driveSpeedScale = 1.0;
  // REAL robot driver controller
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // SIM keyboard (map "Keyboard 0" -> Joystick[0] in the sim GUI)
  private final Joystick simKb = new Joystick(0);

  // Your YAGSL SwerveSubsystem (directory may be "swerve" or "swerve/neo" depending on your deploy folder)
  private final SwerveSubsystem drivebase =                  //TODO need this to instantiate the drivetrain
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // Drive-mode chooser (this is what will show in Elastic)
  private final SendableChooser<Command> driveModeChooser = new SendableChooser<>();
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // Keep track of current so we can swap live
  private Command activeDriveCommand = null;

  public RobotContainer() {
    configureDriveModes(); //TODO need both of these to start up the swerve
    setupAutonomous();

    // Publish chooser to NT/SmartDashboard (Elastic will see this)
    SmartDashboard.putData("Drive Mode", driveModeChooser);
  CommandScheduler.getInstance().schedule(
    Commands.run(() -> {
        // updateDriveMode();

        Translation2d hub = getHubTarget();
        SmartDashboard.putNumber("TargetHub/X", hub.getX());
        SmartDashboard.putNumber("TargetHub/Y", hub.getY());
    }).ignoringDisable(true)
);

    // Run an always-on updater so changing the chooser swaps modes live  //TODO needed to be able to swtich drive in elastic
    CommandScheduler.getInstance().schedule(
        Commands.run(this::updateDriveMode).ignoringDisable(true)
    );

    DriverStation.silenceJoystickConnectionWarning(true);
      // turret.setDefaultCommand(new TrackTurretWithLimelight(turret, vision));

    configureBindings();
    // configureTurretBindings();

    //------------------------- 
    //Autonamous commands call out after they are made so they can be used in pathfinder or ona button bind
    // NamedCommands.registerCommand("AimAndShoot", aimAndShootCommand());
    // NamedCommands.registerCommand("AimAtHub", aimTurretAtHubCommand());
    // NamedCommands.registerCommand("Shoot", shootCommand(() -> vision.getDistanceMetersMostRecent()));
    NamedCommands.registerCommand("Intake On", intakeOnCommand());
    NamedCommands.registerCommand("Intake Off", intakeOffCommand());
    NamedCommands.registerCommand("Deploy Slider", deploySliderOnceCommand());
    NamedCommands.registerCommand("Withdraw Slider", WithdrawSliderOnceCommand());
    NamedCommands.registerCommand("Shoot", shootPoseCommand(null));
    // NamedCommands.registerCommand("Hang Up Timed", hangUpTimedCommand(2.0));
    // NamedCommands.registerCommand("Hang Down Timed", hangDownTimedCommand(2.0));

//     turret.setDefaultCommand(           // gave manual control to operater for turret, not needed
//     Commands.run(() -> {
//         double input = operator.getRightX();

//         if (Math.abs(input) < 0.05) {
//             turret.stop();
//             return;
//         }

//         turret.setOpenLoop(input * 0.5);
//     }, turret)
// );

    // CommandScheduler.getInstance().schedule(
    //     Commands.run(this::updateVisionPose, drivebase).ignoringDisable(false)
    // );

}
 
//command to update pose from the apriltag detected
// private void updateVisionPose() {
//   vision.setRobotOrientation(drivebase.getHeading().getDegrees());
//   var measurementOpt = vision.getLatestVisionMeasurement();
//   if (measurementOpt.isEmpty()) {
//     return;
//   }

//   var measurement = measurementOpt.get();

//   double poseJump =
//       drivebase.getPose().getTranslation().getDistance(measurement.pose.getTranslation());

//   if (poseJump > Constants.VisionConstants.MAX_POSE_JUMP_METERS) {
//     SmartDashboard.putString("Vision/RejectReason", "Pose jump too large");
//     return;
//   }

//   if (Math.abs(drivebase.getGyroRateDegPerSec()) > Constants.VisionConstants.MAX_ROTATION_RATE_DEG_PER_SEC) {
//     SmartDashboard.putString("Vision/RejectReason", "Turning too fast");
//     return;
//   }

//   SmartDashboard.putString("Vision/RejectReason", "ACCEPTED");

//   if (measurement.tagCount >= 2) {
//     drivebase.addVisionMeasurement(
//         measurement.pose,
//         measurement.timestampSec,
//         drivebase.getMultiTagStdDevs());
//   } else {
//     drivebase.addVisionMeasurement(
//         measurement.pose,
//         measurement.timestampSec,
//         drivebase.getSingleTagStdDevs());
//   }
// }

private void setupAutonomous() {
  autoChooser.setDefaultOption("Blue 1", "Blue 1");
  autoChooser.addOption("Blue 2", "Blue 2");
    autoChooser.addOption("MiddleDepot+HumanPlayer", "MiddleDepot+HumanPlayer");
      autoChooser.addOption("Left", "Left");
  autoChooser.addOption("Right", "Right");

  SmartDashboard.putData("Autonomous Mode", autoChooser);
}

//Autonamous commands
private Command intakeOnCommand() {
  return Commands.runOnce(() -> intakeRoller.intake(), intakeRoller);
}

private Command intakeOffCommand() {
  return Commands.runOnce(() -> intakeRoller.stop(), intakeRoller);
}

private Command deploySliderOnceCommand() {
  return Commands.sequence(
      Commands.run(() -> intakeSlider.stow(), intakeSlider).withTimeout(0.6),
      Commands.runOnce(() -> intakeSlider.stop(), intakeSlider)
  );
}
private Command WithdrawSliderOnceCommand() {
  return Commands.sequence(
      Commands.run(() -> intakeSlider.deploy(), intakeSlider).withTimeout(0.6),
      Commands.runOnce(() -> intakeSlider.stop(), intakeSlider)
  );
}

// private Command hangUpTimedCommand(double seconds) {
//   return Commands.sequence(
//       Commands.run(() -> climb.runUp(), climb).withTimeout(2),
//       Commands.runOnce(() -> climb.stop(), climb)
//   );
// }

// private Command hangDownTimedCommand(double seconds) {
//   return Commands.sequence(
//       Commands.run(() -> climb.runDown(), climb).withTimeout(2),
//       Commands.runOnce(() -> climb.stop(), climb)
//   );
// }

private static final Translation2d HUB_BLUE = new Translation2d(4.6, 4.0);
private static final Translation2d HUB_RED  = new Translation2d(12, 4.0); // change later if needed

private Pose2d getStartResetPose() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        return new Pose2d(16, 7.6, Rotation2d.fromDegrees(0.0)); // change later
    }

    return new Pose2d(.35, 0.45, Rotation2d.fromDegrees(0.0));
}

private Pose2d getBackResetPose() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        return new Pose2d(16, .45, Rotation2d.fromDegrees(0.0)); // change later
    }

    return new Pose2d(0.35, 7.6, Rotation2d.fromDegrees(0.0));
}


private Translation2d getHubTarget() {
  var alliance = DriverStation.getAlliance();
  if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
    return HUB_RED;
  }
  return HUB_BLUE;
}
private Pose2d getCenterResetPose() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        return new Pose2d(12.9, 4, Rotation2d.fromDegrees(0.0)); // change later
    }

    return new Pose2d(3.4, 4, Rotation2d.fromDegrees(180.0));
}

// private Command aimAndShootPoseCommand() {
//   return Commands.sequence(
//       Commands.runOnce(() -> {
//     Translation2d hub = getHubTarget();
//     turret.aimAtFieldPoint(hub.getX(), hub.getY(), drivebase.getPose());
// }, turret),

//       Commands.waitUntil(() -> turret.atTarget()).withTimeout(1.8),

//       Commands.parallel(
//           Commands.run(() -> {
//             double distance = drivebase.getPose().getTranslation().getDistance(getHubTarget());
//             flywheel.setPower(flywheel.getPowerForPoseDistance(distance));
//             feeder.feed();
//           }, flywheel, feeder).withTimeout(6.0)

          // Commands.repeatingSequence(
          //     Commands.run(() -> intakePivot.setPower(0.7), intakePivot).withTimeout(0.12),
          //     Commands.run(() -> intakePivot.setPower(-0.10), intakePivot).withTimeout(0.2)
          // ).withTimeout(2.0)
//       ),

//       Commands.runOnce(() -> {
//         flywheel.stop();
//         feeder.stop();
//         intakePivot.stop();
//       }, flywheel, feeder, intakePivot)
//   );
// }












  /** ---------------- SIM keyboard helpers ---------------- */
  private double simForward() {
    int pov = simKb.getPOV(); // -1 none, 0 up, 90 right, 180 down, 270 left
    if (pov == 0) return 1.0;
    if (pov == 180) return -1.0;
    return 0.0;
  }

  private double simStrafe() {
    int pov = simKb.getPOV();
    if (pov == 270) return 1.0;   // left
    if (pov == 90)  return -1.0;  // right
    return 0.0;
  }

  // Rotation in sim (these button numbers depend on the sim keyboard map)
  // If these don't work, check which buttons light up in the sim Joystick[0] panel.
  private double simRot() {
    double rot = 0.0;
    if (simKb.getRawButton(5)) rot += 1.0; // rotate left
    if (simKb.getRawButton(6)) rot -= 1.0; // rotate right
    return rot;
  }

  // /** ---------------- Drive modes + chooser ---------------- */
  private void configureDriveModes() {            //TODO must enable fordrivetrain to work appropriately
    // ----- REAL (Xbox) input streams -----
    SwerveInputStream driveAngularVelocity =
        SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> -driverXbox.getLeftY() * driveSpeedScale,
                () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(() -> driverXbox.getRightX() * driveSpeedScale)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle =
        driveAngularVelocity.copy()
            .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
            .headingWhile(true);

    SwerveInputStream driveRobotOriented =
        driveAngularVelocity.copy()
            .robotRelative(true)
            .allianceRelativeControl(false);

    // ----- SIM (arrow keys) input streams -----
    // Field-oriented, angular velocity control (arrows translate, buttons rotate)
    SwerveInputStream driveAngularVelocitySim =
        SwerveInputStream.of(drivebase.getSwerveDrive(),
                this::simForward,
                this::simStrafe)
            .withControllerRotationAxis(this::simRot)
            .deadband(0.05)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    // “Direct angle” in sim: robot faces the direction you’re translating (nice for arrows)
    SwerveInputStream driveDirectAngleSim =
        driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(this::simStrafe, this::simForward)
            .headingWhile(true);

    SwerveInputStream driveRobotOrientedSim =
        driveAngularVelocitySim.copy()
            .robotRelative(true)
            .allianceRelativeControl(false);

    // ----- Build Commands from streams (YAGSL style) -----
    Command fieldAngularVel = drivebase.driveFieldOriented(driveAngularVelocity);
    Command fieldDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command robotAngularVel = drivebase.driveFieldOriented(driveRobotOriented);
    // Command setpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    Command fieldAngularVelSim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command fieldDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command robotAngularVelSim = drivebase.driveFieldOriented(driveRobotOrientedSim);
    // Command setpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

    // ----- Fill chooser -----
    if (RobotBase.isSimulation()) {
      driveModeChooser.setDefaultOption("Field (Direct Angle) [SIM]", fieldDirectAngleSim);
      driveModeChooser.addOption("Field (Angular Velocity) [SIM]", fieldAngularVelSim);
      driveModeChooser.addOption("Robot (Angular Velocity) [SIM]", robotAngularVelSim);
      // driveModeChooser.addOption("Setpoint Generator [SIM]", setpointGenSim);
    } else {
      driveModeChooser.setDefaultOption("Field (Angular Velocity)", fieldAngularVel);
      driveModeChooser.addOption("Field (Direct Angle)", fieldDirectAngle);
      driveModeChooser.addOption("Robot (Angular Velocity)", robotAngularVel);
      // driveModeChooser.addOption("Setpoint Generator", setpointGen);
    }
  }

  // Runs continuously; when the chooser changes, it swaps the default command live
  private void updateDriveMode() {
    Command selected = driveModeChooser.getSelected();
    if (selected == null || selected == activeDriveCommand) return;

    // Cancel previous default if it was running (safe)
    if (activeDriveCommand != null) {
      activeDriveCommand.cancel();
    }

    drivebase.setDefaultCommand(selected);
    activeDriveCommand = selected;
  }




public Command getAutonomousCommand() {    //TODO must enable for autonamos
  String selected = autoChooser.getSelected();

  if (selected == null) {
    return Commands.none();
  }

  return drivebase.getAutonomousCommand(selected);
}

//   private Command aimAndShootCommand() {
//   return Commands.sequence(
//       Commands.runOnce(() -> {
//         if (vision.hasRecentTarget()) {
//           double targetDeg = turret.getTurretDeg() + vision.getYawDegMostRecent();
//           turret.setTurretAngleDeg(targetDeg);
//         }
//       }, turret),

//       Commands.waitUntil(() ->
//           Math.abs(turret.getTargetTurretDeg() - turret.getTurretDeg()) < 2.0
//       ).withTimeout(2.0),

//       Commands.parallel(
//           Commands.run(() -> {
//             double d = vision.getDistanceMetersMostRecent();
//             double power = (d > 0) ? flywheel.getPowerForDistance(d) : 0.6;
//             flywheel.setPower(power);
//             feeder.feed();
//           }, flywheel, feeder).withTimeout(4.0),

//           Commands.repeatingSequence(
//               Commands.run(() -> intakePivot.setPower(0.7), intakePivot).withTimeout(0.12),
//               Commands.run(() -> intakePivot.setPower(-0.10), intakePivot).withTimeout(0.2)
//           ).withTimeout(4.0)
//       ),

//       Commands.runOnce(() -> {
//         flywheel.stop();
//         feeder.stop();
//         intakePivot.stop();
//       }, flywheel, feeder, intakePivot)
//   );
// }

//   public Command getAutonomousCommand() {
//         // Return the Aim-and-shoot autonomous sequence
//         return new AimAndShootAuton(turret, vision, flywheel, feeder, intakePivot);
//   }
// private Command shootLLCommand(DoubleSupplier distanceSupplier) {
//   return Commands.run(() -> {
//       double distance = distanceSupplier.getAsDouble();
//       SmartDashboard.putNumber("Shot/LLDistance", distance);
//       double power = flywheel.getPowerForLLDistance(distance);
//       SmartDashboard.putNumber("Shot/LLPowerCmd", power);
//       flywheel.setPower(power);
//     }, flywheel)
//     .alongWith(
//         Commands.sequence(
//             Commands.waitSeconds(0.5),
//             Commands.run(() -> feeder.feed(), feeder)
//         )
//     );

// }

private Command shootPoseCommand(DoubleSupplier distanceSupplier) {  //shot command for autonamous with no turret
  return Commands.run(() -> {
      double distance = distanceSupplier.getAsDouble();
      SmartDashboard.putNumber("Shot/PoseDistance", distance);
      double power = flywheel.getPowerForPoseDistance(distance);
      SmartDashboard.putNumber("Shot/PosePowerCmd", power);
      flywheel.setPower(power);
    }, flywheel)
    .alongWith(
        Commands.sequence(
            Commands.waitSeconds(1.5),
            Commands.run(() -> feeder.feed(), feeder)
        )
    );
  }
//     // .alongWith(
//     //     Commands.repeatingSequence(
//     //         Commands.run(() -> intakePivot.setPower(0.7), intakePivot).withTimeout(0.12),
//     //         Commands.run(() -> intakePivot.setPower(-0.10), intakePivot).withTimeout(0.2)
//     //     )
//     // );
// }

// private Command aimTurretAtHubCommand() {
//   return Commands.run(() -> {
// Translation2d hub = getHubTarget();
// turret.aimAtFieldPoint(hub.getX(), hub.getY(), drivebase.getPose());
//   }, turret);
// }

// private Command shootCommand(DoubleSupplier distanceSupplier) {
//   return Commands.run(() -> {
//       double distance = distanceSupplier.getAsDouble();
//       double power = flywheel.getPowerForDistance(distance);
//       flywheel.setPower(power);
//     }, flywheel)

//     .alongWith(
//         Commands.sequence(
//             Commands.waitSeconds(0.5),
//             Commands.run(() -> feeder.feed(), feeder)
//         )
//     );

// }


// private void configureTurretBindings() {

// driverXbox.povUp().onTrue(
//     Commands.runOnce(() -> turret.setTurretAngleDeg(turret.getTurretDeg() + 10), turret)
// );
// driverXbox.povDown().onTrue(
//     Commands.runOnce(() -> turret.setTurretAngleDeg(turret.getTurretDeg() - 10), turret)
// );
// driverXbox.povRight().onTrue(
//     Commands.runOnce(() -> turret.setTurretAngleDeg(turret.getTurretDeg() + 45), turret)
// );
// driverXbox.povLeft().onTrue(
//     Commands.runOnce(() -> turret.setTurretAngleDeg(turret.getTurretDeg() - 45), turret)
// );
// }


  private void configureBindings() {


    driverXbox.povLeft().onTrue(
        Commands.runOnce(() ->  driveSpeedScale = 0.3)
        );
    
    driverXbox.povRight().onTrue(
        Commands.runOnce(() ->  driveSpeedScale = 1)
        );
  // ----------------------------

//    driverXbox.start().onTrue(
//     Commands.runOnce(() -> drivebase.resetOdometry(getStartResetPose()), drivebase)  //commands to reset odometry for turret racking
// );

// driverXbox.back().onTrue(
//     Commands.runOnce(() -> drivebase.resetOdometry(getBackResetPose()), drivebase)
// );

driverXbox.y().onTrue(
    Commands.runOnce(() -> drivebase.resetOdometry(getCenterResetPose()), drivebase)
);

  // INTAKE PIVOT (independent)
  // ----------------------------
  // ----------------------------
  // OPERATOR INTAKE PIVOT MANUAL
  // ----------------------------

  driverXbox.povUp().whileTrue(
      Commands.run(() -> intakeSlider.jogUp(), intakeSlider)
  );
  driverXbox.povUp().onFalse(
      Commands.runOnce(() -> intakeSlider.stop(), intakeSlider)
  );

  driverXbox.povDown().whileTrue(
      Commands.run(() -> intakeSlider.jogDown(), intakeSlider)
  );
  driverXbox.povDown().onFalse(
      Commands.runOnce(() -> intakeSlider.stop(), intakeSlider)
  );




//   driverXbox.x().onTrue(
//     Commands.runOnce(() -> intakePivot.zeroHere(), intakePivot)
// );

// driverXbox.y().onTrue(
//     Commands.runOnce(() -> intakePivot.stow(), intakePivot)
// );

// driverXbox.b().onTrue(
//     Commands.runOnce(() -> intakePivot.feedPosition(), intakePivot)
// );

// driverXbox.a().onTrue(
//     Commands.runOnce(() -> intakePivot.deploy(), intakePivot)
// );

  // ----------------------------
  // INTAKE ROLLERS (independent)
  // ----------------------------

  // HOLD Right Trigger = intake rollers only
  driverXbox.rightTrigger().whileTrue(
      Commands.run(() -> intakeRoller.intake(), intakeRoller)
  );
  driverXbox.rightTrigger().onFalse(
      Commands.runOnce(() -> intakeRoller.stop(), intakeRoller)
  );

  // // HOLD Y = outtake rollers only
  driverXbox.leftTrigger().whileTrue(
      Commands.run(() -> intakeRoller.outtake(), intakeRoller)
  );
  driverXbox.leftTrigger().onFalse(
      Commands.runOnce(() -> intakeRoller.stop(), intakeRoller)
  );

  // ----------------------------
  // FEEDER (independent)
  // ----------------------------

  // HOLD Left Trigger = feeder forward only
//   driverXbox.y().whileTrue(
//       Commands.run(() -> flywheel.setPower(0.30), flywheel)
//   );
//   driverXbox.y().onFalse(
//       Commands.runOnce(() -> flywheel.stop(), flywheel)
//   );

  // HOLD X = feeder reverse only
  // operator.a().whileTrue(
  //     Commands.run(() -> feeder.reverse(), feeder)
  // );
  // operator.a().onFalse(
  //     Commands.runOnce(() -> feeder.stop(), feeder)
  // );

  // ----------------------------
  // SHOOT (flywheel + feeder after 0.5s delay)
  // ----------------------------

  // HOLD Right Bumper = spin flywheel immediately, start feeder after 0.5s
  // operator.rightBumper().whileTrue(
  //     Commands.run(() -> flywheel.shoot(), flywheel)
  //         .alongWith(
  //             Commands.sequence(
  //                 Commands.waitSeconds(0.5),
  //                 Commands.run(() -> feeder.feed(), feeder)
  //             )
  //         )
  // );

  // Release Right Bumper = stop flywheel + stop feeder
// operator.rightBumper().whileTrue(
//     Commands.run(() -> {
//       double distance = vision.getDistanceMetersMostRecent();
//       if (distance <= 0) { flywheel.stop(); return; }
//       flywheel.setPower(flywheel.getPowerForDistance(distance));
//     }, flywheel)
//     .alongWith(
//         Commands.sequence(
//             Commands.waitSeconds(0.5),
//             Commands.run(() -> feeder.feed(), feeder)
//         )
//     )
//     .alongWith(
//         Commands.repeatingSequence(
//             Commands.run(() -> intakePivot.setPower(0.7), intakePivot).withTimeout(0.12),
//             Commands.run(() -> intakePivot.setPower(-0.10), intakePivot).withTimeout(0.2)
//         )
//     )
// );

// operator.rightBumper().whileTrue(
//     Commands.run(() -> {
//       flywheel.setPower(flywheel.getPowerForPose(drivebase.getPose()));
//     }, flywheel)
//     .alongWith(
//         Commands.sequence(
//             Commands.waitSeconds(0.5),
//             Commands.run(() -> feeder.feed(), feeder)
//         )
//     )
//     .alongWith(
//         Commands.repeatingSequence(
//             Commands.run(() -> intakePivot.setPower(0.7), intakePivot).withTimeout(0.12),
//             Commands.run(() -> intakePivot.setPower(-0.10), intakePivot).withTimeout(0.2)
//         )
//     )
// );
// operator.rightBumper().whileTrue(
//     shootLLCommand(() -> vision.getDistanceMetersMostRecent())
// );
driverXbox.leftBumper().whileTrue(
    // shootPoseCommand(() -> {
    //     Translation2d hub = getHubTarget();
    //     return drivebase.getPose().getTranslation().getDistance(hub);
    // })
    Commands.run(() -> flywheel.setPower(.90), flywheel)
        .alongWith(
            Commands.sequence(
                Commands.waitSeconds(1.5),
                Commands.run(() -> feeder.feed(), feeder)
            )
        )
);

// operator.leftTrigger().whileTrue(
//     Commands.run(() -> {
//         Translation2d hub = getHubTarget();
//         turret.aimAtFieldPoint(hub.getX(), hub.getY(), drivebase.getPose());
//     }, turret)
// );


// operator.rightBumper().onFalse(
//     Commands.runOnce(() -> {
//       feeder.stop();
//       flywheel.stop();
//       intakePivot.stop();
//     }, feeder, flywheel, intakePivot)
// );

// driverXbox.povUp().whileTrue(
//     Commands.run(() -> climb.runUp(), climb)
// );
// driverXbox.povUp().onFalse(
//     Commands.runOnce(() -> climb.stop(), climb)
// );

// driverXbox.povDown().whileTrue(
//     Commands.run(() -> climb.runDown(), climb)
// );
// driverXbox.povDown().onFalse(
//     Commands.runOnce(() -> climb.stop(), climb)
// );
// operator.leftBumper().toggleOnTrue(
//     Commands.run(() -> {

// double distance = vision.getDistanceMetersMostRecent();

// if (distance <= 0) {
//   flywheel.stop();
//   feeder.stop();
//   return;
// }

// double power = flywheel.getPowerForDistance(distance);
// flywheel.setPower(power);

//     }, flywheel)
// );

// operator.pov(90).whileTrue(
//     Commands.run(() -> flywheel.setPower(.30), flywheel)
//         .alongWith(
//             Commands.sequence(
//                 Commands.waitSeconds(1.0),
//                 Commands.run(() -> feeder.feed(), feeder)
//             )
//         )
// );

// operator.pov(90).onFalse(
//     Commands.runOnce(() -> {
//       feeder.stop();
//       flywheel.stop();
//     }, feeder, flywheel)
// );

//   // ----------------------------
//   // PANIC STOP (everything)
//   // ----------------------------

  // Press START = stop everything immediately
  driverXbox.start().onTrue(
      Commands.runOnce(() -> {
        // intakePivot.stop();
        // intakeRoller.stop();
        feeder.stop();
        flywheel.stop();
      }, 
      // intakePivot,
       feeder, flywheel)
  );

// operator.leftStick().toggleOnTrue(

// );
  
}


  //  private final LimelightSubsystem vision = new LimelightSubsystem();

}
