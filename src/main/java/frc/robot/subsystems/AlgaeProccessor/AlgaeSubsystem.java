package frc.robot.subsystems.AlgaeProccessor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;;

public class AlgaeSubsystem extends SubsystemBase {
  SparkMax wheelMotor = new SparkMax(MotorConstants.ALGAE_INTAKE_CAN_ID, MotorType.kBrushless);
  SparkMax armMotor = new SparkMax(MotorConstants.ALGAE_INTAKE_CAN_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  public AlgaeSubsystem() {
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    // Pushes new speed to intake wheel motor
  public void setIntakeSpeed(double speed) {
    // Updates intake wheel speed on dashboard
    SmartDashboard.putNumber("Algae Intake Speed: ", speed);
    wheelMotor.set(speed);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

}
