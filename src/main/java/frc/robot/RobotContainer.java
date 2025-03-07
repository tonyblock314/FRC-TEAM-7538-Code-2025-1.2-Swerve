// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Telemetry;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.SwerveConstants;
import frc.robot.commands.AlgaeProcessor.GrabBall;
import frc.robot.commands.Arm.JorkingIt;
import frc.robot.commands.Climber.Lever;
import frc.robot.commands.Elevator.UppyDowny;
import frc.robot.commands.ManualCommands.ManualArm;
import frc.robot.commands.ManualCommands.ManualBalls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeProccessor.AlgaeSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.CoralIntake.CoralSubsystem;
import frc.robot.subsystems.Elevator.ElevatorMotionMagicSubsystemRii;
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

//@SuppressWarnings("unused")
public class RobotContainer {
  
  private final ElevatorMotionMagicSubsystemRii m_elevator = new ElevatorMotionMagicSubsystemRii();
  private final AlgaeSubsystem m_algae = new AlgaeSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final ManualArm downCommand = new ManualArm(m_arm, ArmConstants.ARM_DOWN_POSITION);
  private final ManualArm deliverCommand = new ManualArm(m_arm, ArmConstants.ARM_UP_POSITION);
  private final ManualArm pickUpCommand = new ManualArm(m_arm, ArmConstants.ARM_INTAKE_POSITION);
  private final ManualBalls fondleBalls = new ManualBalls(m_algae, AlgaeIntakeConstants.AUTO_ALGAE_SPEED);
  private final ManualBalls releaseBalls = new ManualBalls(m_algae, AlgaeIntakeConstants.AUTO_ALGAE_SPEED);
  
  private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  CommandXboxController driverController = new CommandXboxController(0);
  XboxController commandsController = new XboxController(1);

  // Sendable Chooser Auto Setup
  private final SendableChooser<Command> autoChooser;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    NamedCommands.registerCommand("Level1", m_elevator.LevelOne());
    NamedCommands.registerCommand("Level2", m_elevator.LevelTwo());
    NamedCommands.registerCommand("Level3", m_elevator.LevelThree());
    NamedCommands.registerCommand("Level4", m_elevator.LevelFour());
    NamedCommands.registerCommand("DeliverCoral", m_algae.deliverAlgae());
    NamedCommands.registerCommand("ArmToDown", downCommand);
    NamedCommands.registerCommand("ArmToDeliver", deliverCommand);
    NamedCommands.registerCommand("ArmToIntake", pickUpCommand);
    NamedCommands.registerCommand("FondleBalls", fondleBalls);
    NamedCommands.registerCommand("FondleBalls", releaseBalls);
    
    autoChooser = AutoBuilder.buildAutoChooser("Im Jaking It");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    
    /* 
    driverController.leftBumper().onTrue(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps)
        .andThen(() -> AngularRate));
    driverController.leftBumper().onFalse(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * speedChooser.getSelected())
        .andThen(() -> AngularRate = MaxAngularRate));
    */
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(-(driverController.getLeftY() * Math.abs(driverController.getLeftY())) * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-(driverController.getLeftX() * Math.abs(driverController.getLeftX())) * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-(driverController.getRightX() * Math.abs(driverController.getRightX())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on start button press
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
     

    drivetrain.registerTelemetry(logger::telemeterize);
    
    //CameraServer.startAutomaticCapture();
    
    m_elevator.setDefaultCommand(new UppyDowny(m_elevator,
      commandsController::getXButtonPressed,
      commandsController::getAButtonPressed,
      commandsController::getBButtonPressed,
      commandsController::getYButtonPressed
    ));
    m_arm.setDefaultCommand(new JorkingIt(m_arm, 
      commandsController::getRightY,
      commandsController::getRightX
    ));
    /* 
    m_coral.setDefaultCommand(new GrabPolePhillip(m_coral,
      commandsController::getLeftBumperButton,
      commandsController::getRightBumperButton
      ));
    */
    m_algae.setDefaultCommand(new GrabBall(m_algae,
      commandsController::getLeftTriggerAxis,
      commandsController::getRightTriggerAxis
      ));
    m_climber.setDefaultCommand(new Lever(m_climber,
      commandsController::getLeftY,
      commandsController::getLeftStickButtonPressed
    ));
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
    //return Commands.print("No autonomous command configured");
  }
}
