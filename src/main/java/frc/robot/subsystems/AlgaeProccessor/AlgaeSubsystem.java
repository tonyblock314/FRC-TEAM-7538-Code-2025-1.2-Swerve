package frc.robot.subsystems.AlgaeProccessor;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;;

public class AlgaeSubsystem extends SubsystemBase {
  SparkMax fondlingMotor = new SparkMax(MotorConstants.FONDLING_MOTOR_CAN_ID ,MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  public AlgaeSubsystem() {
    fondlingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    // Pushes new speed to intake wheel motor
  public void setFondleSpeed(double speed) {
    // Updates intake wheel speed on dashboard
    SmartDashboard.putNumber("Algae Intake Speed: ", speed);
    fondlingMotor.set(speed);
  }

  public Command deliverAlgae() {
    return this.startEnd(() -> this.setFondleSpeed(0.3), () -> this.setFondleSpeed(0));
  }
}
