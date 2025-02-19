// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class UppyDowny extends Command {
  private ElevatorSubsystem m_subsystem;
  private BooleanSupplier xPressed, aPressed, bPressed, yPressed;
  private DoubleSupplier rightTilt;

  /** Creates a new UppyDowny. */
  public UppyDowny(ElevatorSubsystem subsystem, BooleanSupplier xPressed, BooleanSupplier aPressed, BooleanSupplier bPressed, BooleanSupplier yPressed, DoubleSupplier rightTilt) {
    this.m_subsystem = subsystem;
    this.xPressed = xPressed;
    this.aPressed = aPressed;
    this.bPressed = bPressed;
    this.yPressed = yPressed;
    this.rightTilt = rightTilt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (aPressed.getAsBoolean()) {
      m_subsystem.elevatorPosition(ElevatorConstants.LEVEL_1);
    } else if (xPressed.getAsBoolean()) {
      m_subsystem.elevatorPosition(ElevatorConstants.LEVEL_2);
    } else if (bPressed.getAsBoolean()) {
      m_subsystem.elevatorPosition(ElevatorConstants.LEVEL_3);
    } else if (yPressed.getAsBoolean()) {
      m_subsystem.elevatorPosition(ElevatorConstants.LEVEL_4);
    }

    double eSpeed;
    if(Math.abs(rightTilt.getAsDouble()) < OperatorConstants.RIGHT_Y_DEADBAND) {
      eSpeed = 0;
    } else {
      eSpeed = rightTilt.getAsDouble();
    }

    eSpeed = (eSpeed) * ElevatorConstants.ELEVATOR_SCALING_FACTOR;

    m_subsystem.setElevatorSpeed(eSpeed);

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
