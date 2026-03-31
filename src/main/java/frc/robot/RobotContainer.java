// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.shooter.SetPositionShootCommand;
import frc.robot.Constants.OperatorConstants.LogiConstants;
import frc.robot.Constants.OperatorConstants.PikachuConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandJoystick driverJoystick = new CommandJoystick(0);
  final         CommandJoystick operatorJoystick = new CommandJoystick(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  //private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

                                            
  //private final Vision vision = new Vision(drivebase.getPose(), drivebase.field)

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverJoystick.getRawAxis(LogiConstants.LEFT_Y_AXIS) * -1,
                                                                () -> driverJoystick.getRawAxis(LogiConstants.LEFT_X_AXIS) * -1)
                                                            .withControllerRotationAxis(() -> driverJoystick.getRawAxis(LogiConstants.RIGHT_X_AXIS))
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverJoystick.getRawAxis(LogiConstants.RIGHT_Y_AXIS),
                                                                                            () -> driverJoystick.getRawAxis(LogiConstants.RIGHT_X_AXIS))
                                                           .headingWhile(true);

  /*
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverJoystick.getRawAxis(LogiConstants.LEFT_Y_AXIS),
                                                                        () -> -driverJoystick.getRawAxis(LogiConstants.LEFT_X_AXIS))
                                                                    .withControllerRotationAxis(() -> driverJoystick.getRawAxis(
                                                                        LogiConstants.RIGHT_X_AXIS))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      LogiConstants.RIGHT_X_AXIS) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      LogiConstants.RIGHT_X_AXIS) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    registerPathPlannerCommands();

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerPathPlannerCommands () {
    //NamedCommands.registerCommand("Throw_cheese", new SetPositionShootCommand(shooterSubsystem, drivebase));
    NamedCommands.registerCommand("Eat_cheese", intakeSubsystem.startMotor());
    NamedCommands.registerCommand("Hide_tray", intakeSubsystem.stopMotor());
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle)>;
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
    //    driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverJoystick.button(LogiConstants.BACK_BUTTON).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverJoystick.button(LogiConstants.START_BUTTON).whileTrue(drivebase.sysIdDriveMotorCommand());
      //driverJoystick.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                               () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      

//      driverJoystick.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      driverJoystick.button(0).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverJoystick.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverJoystick.button(0).onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverJoystick.button(LogiConstants.START_BUTTON).onTrue(Commands.runOnce(drivebase::sysIdDriveMotorCommand));
      //driverJoystick.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverJoystick.button(LogiConstants.B_BUTTON).onTrue(intakeSubsystem.startMotor());


      operatorJoystick.button(LogiConstants.Y_BUTTON).onTrue(intakeSubsystem.startMotor()).onFalse(intakeSubsystem.stopMotor());
      
      //driverJoystick.back().whileTrue(drivebase.centerModulesCommand());
      //driverJoystick.leftBumper().onTrue(Commands.none());
      //driverJoystick.rightBumper().onTrue(Commands.none());
    } else
    {
      operatorJoystick.button(LogiConstants.A_BUTTON).onTrue(intakeSubsystem.startMotor())/*.onTrue(shooterSubsystem.startIndexerMotorCommand())*/.onFalse(intakeSubsystem.stopMotor())/*.onFalse(shooterSubsystem.stopIndexerMotorCommand())*/;

      operatorJoystick.button(LogiConstants.B_BUTTON).onTrue(shooterSubsystem.startIndexerMotorCommand()).onTrue(shooterSubsystem.startFlywheelsCommand());//.onFalse(shooterSubsystem.stopIndexerMotorCommand()).onFalse(shooterSubsystem.stopFlywheelsCommand());
    //   driv.erJoystick.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverJoystick.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //   driverJoystick.start().whileTrue(Commands.none());
    //   driverJoystick.back().whileTrue(Commands.none());
    //   driverJoystick.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverJoystick.rightBumper().onTrue(Commands.none());
    //   //driverJoystick.y().onTrue(drivebase.driveToDistanceCommandDefer(drivebase::getPose, 2, 14));
    //   driverJoystick.y().whileTrue(drivebase.driveForward());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
