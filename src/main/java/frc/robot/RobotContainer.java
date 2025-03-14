// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //final CommandJoystick driverJoystick = new CommandJoystick(0);
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final GripperSubsystem gripper = new GripperSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   * Transform Controller inputs into workable Chassis speeds.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream
    .of(drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY() * -1,
        () -> driverXbox.getLeftX() * -1)              // Axis which give the desired translational angle and speed.
    .withControllerRotationAxis(driverXbox::getRightX) // Axis which give the desired angular velocity.
    .deadband(OperatorConstants.DEADBAND)              // Controller deadband
    .scaleTranslation(0.8)            // Scaled controller translation axis
    .allianceRelativeControl(true);            // Alliance relative controls.

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   * Copy the stream so further changes do not affect driveAngularVelocity
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity
      .copy()
      .withControllerHeadingAxis(driverXbox::getRightX, 
                                 driverXbox::getRightY) // Axis which give the desired heading angle using trigonometry.
      .headingWhile(true);                 // Enable heading based control.

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity
      .copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream
    .of(drivebase.getSwerveDrive(),
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX())
    .withControllerRotationAxis(
        () -> driverXbox.getRawAxis(2))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
      .copy()
      .withControllerHeadingAxis(
        () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
        () -> Math.cos(driverXbox.getRawAxis(2) *Math.PI) *(Math.PI *2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set default command(s)
    elevator.setDefaultCommand(elevator.setElevatorHeight(0));
    // add code to set the default for the gripper

    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } 
    else {
      // Do we need to check whether we're in teleOp mode?
      
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      
      // Map the Xbox back button to reset gyro
      driverXbox.back().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

      // Map the Xbox start button to reset gyro
      driverXbox.start().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

      // Map the Xbox A button to move the elevator to position 0
      driverXbox.a().whileTrue(elevator.setElevatorHeight(4));

      // Map the Xbox Y button to move the elevator to position 1
      driverXbox.b().whileTrue(elevator.setElevatorHeight(4));

      // Map the Xbox Y button to move the elevator to position 2
      driverXbox.y().whileTrue(elevator.setElevatorHeight(4));

      // Map the Xbox X button to ...?
      //driverXbox.x().whileTrue(gripper.setSomething());

      // Map the Xbox Left Bumper button to ...?
      //driverXbox.leftBumper().onTrue(gripper.setSomething());

      // Map the Xbox Right Bumper button to ...?
      //driverXbox.rightBumper().onTrue(gripper.setSomething());
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } 
    //else {
      //DriverStation.isTeleop();
      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      //driverXbox.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      //driverXbox.start().whileTrue(Commands.none());
      //driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.rightBumper().onTrue(Commands.none());
    //}
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}