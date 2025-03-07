// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

/** Add your docs here. */
public class ElevatorMotionMagicSubsystemRii extends SubsystemBase {

  //WPI_VictorSPX Winch = new WPI_VictorSPX(Constants.TOP_WINCH_1_CAN_ID);
  //TalonFX talon = new TalonFX(Constants.TOP_WINCH_1_CAN_ID);
  TalonFX elevatorMotorL = new TalonFX(MotorConstants.ELEVATOR_CAN_ID_LEFT);
  TalonFX elevatorMotorR = new TalonFX(MotorConstants.ELEVATOR_CAN_ID_RIGHT);
  MotorOutputConfigs brakeMode = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
  Double kG, kS, kV, kA, kP, kI, kD;

  public ElevatorMotionMagicSubsystemRii() {
    var slot0Configs = new Slot0Configs();
    var talonFXConfigs = new TalonFXConfiguration();
    slot0Configs.kG = ElevatorConstants.kG;
    slot0Configs.kS = ElevatorConstants.kS;
    slot0Configs.kV = ElevatorConstants.kV;
    slot0Configs.kA = ElevatorConstants.kA;
    slot0Configs.kP = ElevatorConstants.kP;
    slot0Configs.kI = ElevatorConstants.kI;
    slot0Configs.kD = ElevatorConstants.kD;


    // Set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CRUISEVELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = ElevatorConstants.JERK;

    elevatorMotorL.getConfigurator().apply(talonFXConfigs);
    elevatorMotorR.getConfigurator().apply(talonFXConfigs);
    elevatorMotorL.getConfigurator().apply(motionMagicConfigs);
    elevatorMotorR.getConfigurator().apply(motionMagicConfigs);
    elevatorMotorL.getConfigurator().apply(slot0Configs);
    elevatorMotorL.getConfigurator().apply(brakeMode);
    elevatorMotorR.getConfigurator().apply(slot0Configs);
    elevatorMotorR.getConfigurator().apply(brakeMode);
  }
/* 
  private void updateElevatorPIDFromDashboard() {
    kG = SmartDashboard.getNumber("Elevator kG", kG);
    kS = SmartDashboard.getNumber("Elevator kS", kS);
    kV = SmartDashboard.getNumber("Elevator kV", kV);
    kP = SmartDashboard.getNumber("Elevator kP", kP);
    kI = SmartDashboard.getNumber("Elevator kI", kI);
    kD = SmartDashboard.getNumber("Elevator kD", kD);
  }
  */

  public void elevatorPosition(double position) {
    //updateElevatorPIDFromDashboard();
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    elevatorMotorL.setControl(m_request.withPosition(position));
    elevatorMotorR.setControl(new Follower(elevatorMotorL.getDeviceID(), true));
  }

  public void setElevatorSpeed(double speed) {
    SmartDashboard.putNumber("Intake Speed: ", speed);
    elevatorMotorL.set(-speed);
    elevatorMotorR.set(speed);
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
    return this.startEnd(() -> this.elevatorPosition(ElevatorConstants.SOURCE_LEVEL), () -> this.elevatorPosition(ElevatorConstants.SOURCE_LEVEL));
  }

  
}
