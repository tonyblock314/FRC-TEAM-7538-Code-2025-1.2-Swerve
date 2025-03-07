// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class JorkingIt extends Command {
  /** Creates a new ClimberSubsystem. */
  private ArmSubsystem m_subsystem;
  private DoubleSupplier rightY, rightX;
  //private BooleanSupplier upPressed, rightPressed, downPressed, leftPressed;

  public JorkingIt(ArmSubsystem subsystem, DoubleSupplier rightY, DoubleSupplier rightX) {
    this.m_subsystem = subsystem;
    this.rightY = rightY;
    this.rightX = rightX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rightY.getAsDouble() > 0.8) {
      m_subsystem.armPosition(ArmConstants.ARM_UP_POSITION);
    } else if (rightY.getAsDouble() < -0.8) {
      m_subsystem.armPosition(ArmConstants.ARM_DOWN_POSITION);
    } else if (rightX.getAsDouble() > 0.8) {
      m_subsystem.armPosition(ArmConstants.ARM_INTAKE_POSITION);
    } else if (rightX.getAsDouble() < -0.8) {
      m_subsystem.armPosition(ArmConstants.ARM_INTAKE_GROUND_POSITION);
    }
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
