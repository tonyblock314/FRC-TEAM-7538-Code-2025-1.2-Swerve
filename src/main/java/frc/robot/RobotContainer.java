// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.SwerveConstants;
import frc.robot.commands.AlgaeProcessor.GrabBall;
import frc.robot.commands.Climber.Lever;
import frc.robot.commands.CoralIntake.GrabPole;
import frc.robot.commands.Elevator.UppyDowny;
import frc.robot.subsystems.AlgaeProccessor.AlgaeSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.CoralIntake.CoralSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDrive.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final AlgaeSubsystem m_algae = new AlgaeSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  public final CommandSwerveDrivetrain drivetrain = SwerveConstants.createDrivetrain();

  
  private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);


  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverController = new XboxController(0);
  XboxController commandsController = new XboxController(1);

  // Sendable Chooser Auto Setup
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("levelOne", m_elevator.LevelOne());
    NamedCommands.registerCommand("levelTwo", m_elevator.LevelTwo());
    NamedCommands.registerCommand("levelThree", m_elevator.LevelThree());
    NamedCommands.registerCommand("levelFour", m_elevator.LevelFour());
    NamedCommands.registerCommand("deliverCoral", m_coral.deliverCoral());
    NamedCommands.registerCommand("armToDeliver", m_coral.armToDeliver());
    NamedCommands.registerCommand("armToIntake", m_coral.armToPickup());

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );

    if (driverController.getAButton()) {
      drivetrain.applyRequest(() -> brake);
    }

    if (driverController.getBButton()) {
      drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX())));
    }

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    if (driverController.getBackButton() && (driverController.getYButton())) { 
      drivetrain.sysIdDynamic(Direction.kForward);
    }
    if (driverController.getBackButton() && (driverController.getXButton())) {
      drivetrain.sysIdDynamic(Direction.kReverse);
    }
    if (driverController.getStartButton() && (driverController.getYButton())) {
      drivetrain.sysIdQuasistatic(Direction.kForward);
    }
    if (driverController.getStartButton() && (driverController.getXButton())) {
      drivetrain.sysIdQuasistatic(Direction.kReverse);
    }
       
    // reset the field-centric heading on left bumper press
    if (driverController.getLeftBumper()) {
      drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
    }

    drivetrain.registerTelemetry(logger::telemeterize);
    
    CameraServer.startAutomaticCapture();

    m_elevator.setDefaultCommand(new UppyDowny(m_elevator,
      commandsController::getXButtonPressed,
      commandsController::getAButtonPressed,
      commandsController::getBButtonPressed,
      commandsController::getYButtonPressed,
      commandsController::getRightY
    ));
    m_coral.setDefaultCommand(new GrabPole(m_coral,
      commandsController::getLeftBumperButtonPressed,
      commandsController::getRightBumperButtonPressed
      ));
    m_algae.setDefaultCommand(new GrabBall(m_algae,
      commandsController::getLeftTriggerAxis,
      commandsController::getRightTriggerAxis
      ));
    m_climber.setDefaultCommand(new Lever(m_climber,
    commandsController::getLeftStickButtonPressed,
    commandsController::getRightStickButtonPressed
    ));
    // Configure the trigger bindings
    configureBindings();

    //drivebase.setupPathPlanner();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.setDefaultOption("Auto 2", getAutonomousCommand());
    autoChooser.addOption("LeftAlgaePath", getAutonomousCommand());
    autoChooser.addOption("LeftAlgaePath", getAutonomousCommand());
    autoChooser.addOption("LeftAuto 1", getAutonomousCommand());
    autoChooser.addOption("LeftAuto 2", getAutonomousCommand());
    autoChooser.addOption("LeftAuto 3", getAutonomousCommand());
    autoChooser.addOption("MiddleAlgaePath", getAutonomousCommand());
    autoChooser.addOption("RightAlgaePath", getAutonomousCommand());
    autoChooser.addOption("RightAuto 1", getAutonomousCommand());
    autoChooser.addOption("RightAuto 2", getAutonomousCommand());
    autoChooser.addOption("RightAuto 3", getAutonomousCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
