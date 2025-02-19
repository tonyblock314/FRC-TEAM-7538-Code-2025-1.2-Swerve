// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralIntake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralIntakeConstants;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CoralIntake.CoralSubsystem;

public class GrabPole extends Command {
  private CoralSubsystem m_subsystem;
  private BooleanSupplier leftPressed, rightPressed;

  /** Creates a new GrabPole. */
  public GrabPole(CoralSubsystem subsystem, BooleanSupplier leftPressed, BooleanSupplier rightPressed) {
    this.m_subsystem = subsystem;
    this.leftPressed = leftPressed;
    this.rightPressed = rightPressed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inSpeed = CoralIntakeConstants.CORAL_INTAKE_SPEED;
    double outSpeed = -CoralIntakeConstants.CORAL_INTAKE_SPEED;

    if (leftPressed.getAsBoolean()) {
      m_subsystem.setIntakeSpeed(outSpeed);
    } else if (rightPressed.getAsBoolean()) {
      m_subsystem.setIntakeSpeed(inSpeed);
    } else {
      m_subsystem.setIntakeSpeed(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
