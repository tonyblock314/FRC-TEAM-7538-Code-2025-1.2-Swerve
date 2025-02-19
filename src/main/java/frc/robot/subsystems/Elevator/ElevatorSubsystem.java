// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase {

  //WPI_VictorSPX Winch = new WPI_VictorSPX(Constants.TOP_WINCH_1_CAN_ID);
  //TalonFX talon = new TalonFX(Constants.TOP_WINCH_1_CAN_ID);
  TalonFX elevatorMotorL = new TalonFX(MotorConstants.ELEVATOR_CAN_ID_LEFT);
  TalonFX elevatorMotorR = new TalonFX(MotorConstants.ELEVATOR_CAN_ID_RIGHT);
  MotorOutputConfigs brakeMode = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

  public ElevatorSubsystem() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = 9.8; // Output needed to overcome gravity
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    elevatorMotorL.getConfigurator().apply(slot0Configs);
    elevatorMotorL.getConfigurator().apply(brakeMode);
    elevatorMotorR.getConfigurator().apply(slot0Configs);
    elevatorMotorR.getConfigurator().apply(brakeMode);
  }

  private void motorControl(int position, PositionVoltage m_request) {
    elevatorMotorL.setControl(m_request.withPosition(position));
    elevatorMotorR.setControl(new Follower(elevatorMotorL.getDeviceID(), true));
  }

  public void elevatorPosition(int position) {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    //left.setControl(m_request.withPosition(position));
    //right.setControl(m_request.withPosition(-position));
    motorControl(position, m_request);
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotorL.set(speed);
    elevatorMotorR.set(-speed);
  }

  public Command LevelOne() {
    return this.startEnd(() -> this.elevatorPosition(ElevatorConstants.LEVEL_1), () -> this.elevatorPosition(ElevatorConstants.LEVEL_1));
  }

  public Command LevelTwo() {
    return this.startEnd(() -> this.elevatorPosition(ElevatorConstants.LEVEL_2), () -> this.elevatorPosition(ElevatorConstants.LEVEL_2));
  }

  public Command LevelThree() {
    return this.startEnd(() -> this.elevatorPosition(ElevatorConstants.LEVEL_3), () -> this.elevatorPosition(ElevatorConstants.LEVEL_3));
  }

  public Command LevelFour() {
    return this.startEnd(() -> this.elevatorPosition(ElevatorConstants.LEVEL_4), () -> this.elevatorPosition(ElevatorConstants.LEVEL_4));
  }

  
}
