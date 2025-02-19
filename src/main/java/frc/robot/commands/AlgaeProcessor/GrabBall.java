// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeProcessor;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeProccessor.AlgaeSubsystem;

public class GrabBall extends Command {
  private AlgaeSubsystem m_subsystem;
  private DoubleSupplier rightDepressed;
  private DoubleSupplier leftDepressed;

  /** Creates a new AlgaeSubsystem. */
  public GrabBall(AlgaeSubsystem subsystem, DoubleSupplier leftDepressed, DoubleSupplier rightDepressed) {
    this.m_subsystem = subsystem;
    this.rightDepressed = rightDepressed;
    this.leftDepressed = leftDepressed;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right;
    double left;

    if(rightDepressed.getAsDouble() < OperatorConstants.RT_DEADBAND) {
      right = 0;
    } else {
      right = rightDepressed.getAsDouble();
    }
    if(leftDepressed.getAsDouble() < OperatorConstants.LT_DEADBAND) {
      left = 0;
    } else {
      left = leftDepressed.getAsDouble();
    }

    double intakeSpeed = (right - left);

    m_subsystem.setIntakeSpeed(intakeSpeed);

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
