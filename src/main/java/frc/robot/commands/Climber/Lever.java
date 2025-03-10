// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
//import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class Lever extends Command {
  /** Creates a new ClimberSubsystem. */
  private ClimberSubsystem m_subsystem;
  //private BooleanSupplier downPressed, upPressed;
  private DoubleSupplier leftStickY;
  public Lever(ClimberSubsystem subsystem, DoubleSupplier leftStickY, BooleanSupplier leftStickPressed) {
    this.m_subsystem = subsystem;
    this.leftStickY = leftStickY;
    //this.upPressed = downPressed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    if(Math.abs(leftStickY.getAsDouble()) < OperatorConstants.LEFT_Y_DEADBAND) {
      speed = 0;
    } else {
      speed = leftStickY.getAsDouble() * ClimberConstants.CLIMBER_SCALING_FACTOR;
    }
    m_subsystem.manualOutput(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
