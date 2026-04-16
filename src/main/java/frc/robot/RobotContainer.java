package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.Intake.IntakeRollerSubsystem;
import frc.robot.subsystems.Intake.IntakeSliderSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * RobotContainer is the central wiring class for the robot.
 *
 * This class is responsible for:
 * - creating subsystems
 * - creating shared commands
 * - configuring driver controls
 * - configuring autonomous choices
 * - selecting drive modes
 *
 * Detailed mechanism actions are placed in RobotCommands.
 */
public class RobotContainer {

  // ==================== Subsystems ====================

  private final IntakeSliderSubsystem intakeSlider = new IntakeSliderSubsystem();
  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // ==================== Controllers ====================

  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorXbox = new CommandXboxController(1);

  // Keyboard input for simulation
  private final Joystick simKeyboard = new Joystick(0);

  // ==================== Shared Commands ====================

  private final RobotCommands robotCommands =
      new RobotCommands(intakeSlider, intakeRoller, feeder, flywheel);

  // ==================== Dashboard Choosers ====================

  private final SendableChooser<Command> driveModeChooser = new SendableChooser<>();
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  private Command activeDriveCommand = null;

  // Driver speed scale for precision mode
  private double driveSpeedScale = 1.0;

  public RobotContainer() {
    configureDriveModes();
    configureAutonomousChooser();
    registerNamedCommands();
    configureBindings();

    SmartDashboard.putData("Drive Mode", driveModeChooser);
    SmartDashboard.putData("Autonomous Mode", autoChooser);

    // Continuously update live-selected drive mode from dashboard
    CommandScheduler.getInstance().schedule(
        Commands.run(this::updateDriveMode).ignoringDisable(true));

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  // ==================== Autonomous Setup ====================

  private void configureAutonomousChooser() {
    autoChooser.setDefaultOption("Blue 1", "Blue 1");
    autoChooser.addOption("Blue 2", "Blue 2");
    autoChooser.addOption("MiddleDepot+HumanPlayer", "MiddleDepot+HumanPlayer");
    autoChooser.addOption("Left", "Left");
    autoChooser.addOption("Right", "Right");
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake On", robotCommands.intakeOnCommand());
    NamedCommands.registerCommand("Intake Off", robotCommands.intakeOffCommand());
    NamedCommands.registerCommand("Deploy Intake", robotCommands.deployIntakeCommand());
    NamedCommands.registerCommand("Stow Intake", robotCommands.stowIntakeCommand());

    // Example fixed shot command for autonomous use
    NamedCommands.registerCommand("Shoot", robotCommands.fixedShotCommand(0.90));
  }

  public Command getAutonomousCommand() {
    String selectedAuto = autoChooser.getSelected();

    if (selectedAuto == null) {
      return Commands.none();
    }

    return drivebase.getAutonomousCommand(selectedAuto);
  }

  // ==================== Drive Mode Setup ====================

  /**
   * Helper for sim forward/backward movement using keyboard POV.
   */
  private double simForward() {
    int pov = simKeyboard.getPOV();
    if (pov == 0) return 1.0;
    if (pov == 180) return -1.0;
    return 0.0;
  }

  /**
   * Helper for sim strafing using keyboard POV.
   */
  private double simStrafe() {
    int pov = simKeyboard.getPOV();
    if (pov == 270) return 1.0;
    if (pov == 90) return -1.0;
    return 0.0;
  }

  /**
   * Helper for sim rotation using mapped keyboard buttons.
   */
  private double simRotation() {
    double rotation = 0.0;
    if (simKeyboard.getRawButton(5)) rotation += 1.0;
    if (simKeyboard.getRawButton(6)) rotation -= 1.0;
    return rotation;
  }

  /**
   * Creates available driving modes for real robot and simulation.
   */
  private void configureDriveModes() {
    SwerveInputStream driveAngularVelocity =
        SwerveInputStream.of(
                drivebase.getSwerveDrive(),
                () -> -driverXbox.getLeftY() * driveSpeedScale,
                () -> -driverXbox.getLeftX() * driveSpeedScale)
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

    SwerveInputStream driveAngularVelocitySim =
        SwerveInputStream.of(
                drivebase.getSwerveDrive(),
                this::simForward,
                this::simStrafe)
            .withControllerRotationAxis(this::simRotation)
            .deadband(0.05)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngleSim =
        driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(this::simStrafe, this::simForward)
            .headingWhile(true);

    SwerveInputStream driveRobotOrientedSim =
        driveAngularVelocitySim.copy()
            .robotRelative(true)
            .allianceRelativeControl(false);

    Command fieldAngularVel = drivebase.driveFieldOriented(driveAngularVelocity);
    Command fieldDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command robotAngularVel = drivebase.driveFieldOriented(driveRobotOriented);

    Command fieldAngularVelSim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command fieldDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command robotAngularVelSim = drivebase.driveFieldOriented(driveRobotOrientedSim);

    if (RobotBase.isSimulation()) {
      driveModeChooser.setDefaultOption("Field (Direct Angle) [SIM]", fieldDirectAngleSim);
      driveModeChooser.addOption("Field (Angular Velocity) [SIM]", fieldAngularVelSim);
      driveModeChooser.addOption("Robot (Angular Velocity) [SIM]", robotAngularVelSim);
    } else {
      driveModeChooser.setDefaultOption("Field (Angular Velocity)", fieldAngularVel);
      driveModeChooser.addOption("Field (Direct Angle)", fieldDirectAngle);
      driveModeChooser.addOption("Robot (Angular Velocity)", robotAngularVel);
    }
  }

  /**
   * Updates the active default drive command when dashboard selection changes.
   */
  private void updateDriveMode() {
    Command selected = driveModeChooser.getSelected();
    if (selected == null || selected == activeDriveCommand) {
      return;
    }

    if (activeDriveCommand != null) {
      activeDriveCommand.cancel();
    }

    drivebase.setDefaultCommand(selected);
    activeDriveCommand = selected;
  }

  // ==================== Pose Reset Helpers ====================

  private Pose2d getCenterResetPose() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return new Pose2d(12.9, 4.0, Rotation2d.fromDegrees(0.0));
    }

    return new Pose2d(3.4, 4.0, Rotation2d.fromDegrees(180.0));
  }

  // ==================== Controller Bindings ====================

  private void configureBindings() {
    // Precision drive mode
    driverXbox.povLeft().onTrue(Commands.runOnce(() -> driveSpeedScale = 0.75)); //turns the speed to 30% when the left POV is pressed
    driverXbox.povRight().onTrue(Commands.runOnce(() -> driveSpeedScale = 1.0)); //turns the speed to 100% when the right POV is pressed

    // Reset odometry to known center pose
    driverXbox.y().onTrue(
        Commands.runOnce(() -> drivebase.resetOdometry(getCenterResetPose()), drivebase));
        
        // Intake pivot manual jog
    driverXbox.povUp().whileTrue(
        Commands.run(intakeSlider::jogUp, intakeSlider));
    driverXbox.povUp().onFalse(
        Commands.runOnce(intakeSlider::stop, intakeSlider));

    driverXbox.povDown().whileTrue(
        Commands.run(intakeSlider::jogDown, intakeSlider));
    driverXbox.povDown().onFalse(
        Commands.runOnce(intakeSlider::stop, intakeSlider));


    // // Intake slider step position control
    // driverXbox.a().onTrue(
    //     Commands.runOnce(intakeSlider::stepForward, intakeSlider));

    // driverXbox.b().onTrue(
    //     Commands.runOnce(intakeSlider::stepBackward, intakeSlider));

    // Intake slider preset positions
    driverXbox.x().onTrue(
        Commands.runOnce(intakeSlider::retract, intakeSlider));

    driverXbox.b().onTrue(
        Commands.runOnce(intakeSlider::extend, intakeSlider));

    // Optional preset for a middle/feed position
    driverXbox.rightBumper().onTrue(
        Commands.runOnce(intakeSlider::feedPosition, intakeSlider));

        // Zero the intake slider at its current physical position
    driverXbox.back().onTrue(
        Commands.runOnce(intakeSlider::zeroHere, intakeSlider));
    
    // Intake rollers
    driverXbox.rightTrigger().whileTrue(
        Commands.run(intakeRoller::intake, intakeRoller)); //intakes game pieces when the right trigger is held
    driverXbox.rightTrigger().onFalse(
        Commands.runOnce(intakeRoller::stop, intakeRoller)); //stops the intake rollers when the right trigger is released

    driverXbox.leftTrigger().whileTrue(
        Commands.run(intakeRoller::outtake, intakeRoller)); //outtakes game pieces when the left trigger is held
    driverXbox.leftTrigger().onFalse(
        Commands.runOnce(intakeRoller::stop, intakeRoller)); //stops the intake rollers when the left trigger is released

    // Shooter
    // driverXbox.leftBumper().onTrue(robotCommands.shootRPMCommand(5400)); //shoots with a fixed power of 90% when the left bumper is held
    // driverXbox.leftBumper().onFalse(robotCommands.stopShooterCommand()); //stops the shooter when the left bumper is released
    // driverXbox.b().onTrue(robotCommands.shootRPMCommand(4500)); //shoots with a fixed power of 90% when the left bumper is held
    // driverXbox.a().onTrue(robotCommands.shootRPMCommand(4000)); //shoots with a fixed power of 90% when the left bumper is held


    // Panic stop
    driverXbox.start().onTrue(robotCommands.panicStopCommand()); //stops all mechanisms when the start button is pressed






    operatorXbox.start().onTrue(robotCommands.panicStopCommand()); //stops all mechanisms when the start button is pressed

    operatorXbox.a().onTrue(robotCommands.shootRPMCommand(4100)); //
    operatorXbox.b().onTrue(robotCommands.shootRPMCommand(4400)); //layup about half to full robot length away
    operatorXbox.x().onTrue(robotCommands.shootRPMCommand(4700)); //
    operatorXbox.y().onTrue(robotCommands.shootRPMCommand(5800)); // theoretical pass
    operatorXbox.rightBumper().onTrue(robotCommands.shootRPMCommand(5400)); // 3 Point about 12 feet



  }
}