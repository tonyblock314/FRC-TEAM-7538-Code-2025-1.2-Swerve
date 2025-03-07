package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;;

public class ArmSubsystem extends SubsystemBase {
  TalonFX jorkingMotor = new TalonFX(MotorConstants.ALGAE_JORKING_IT_CAN_ID);

  MotorOutputConfigs brakeMode = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
  Double kG, kS, kV, kA, kP, kI, kD;

  public ArmSubsystem() {
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

    jorkingMotor.getConfigurator().apply(talonFXConfigs);
    jorkingMotor.getConfigurator().apply(motionMagicConfigs);
    jorkingMotor.getConfigurator().apply(slot0Configs);
    jorkingMotor.getConfigurator().apply(brakeMode);
  }

  public void armPosition(double position) {
    //updateElevatorPIDFromDashboard();
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    jorkingMotor.setControl(m_request.withPosition(position));
  }

}
