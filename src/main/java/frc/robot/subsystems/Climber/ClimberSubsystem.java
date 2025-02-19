package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX climberMotorL = new TalonFX(MotorConstants.CLIMBER_CAN_ID_LEFT);
  TalonFX climberMotorR = new TalonFX(MotorConstants.CLIMBER_CAN_ID_RIGHT);
  TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  MotorOutputConfigs brakeMode = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
  DutyCycleOut outputL = new DutyCycleOut(0);
  DutyCycleOut outputR = new DutyCycleOut(0);

  public ClimberSubsystem() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = 9.8; // Output needed to overcome gravity
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    climberMotorL.getConfigurator().apply(slot0Configs);
    climberMotorL.getConfigurator().apply(brakeMode);
    climberMotorR.getConfigurator().apply(slot0Configs);
    climberMotorR.getConfigurator().apply(brakeMode);
  }

  private void motorControl(int position, PositionVoltage m_request) {
    climberMotorL.setControl(m_request.withPosition(position));
    climberMotorR.setControl(new Follower(climberMotorL.getDeviceID(), false));
  }

  public void leverPosition(int position) {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    //left.setControl(m_request.withPosition(position));
    //right.setControl(m_request.withPosition(-position));
    motorControl(position, m_request);
  }

}
