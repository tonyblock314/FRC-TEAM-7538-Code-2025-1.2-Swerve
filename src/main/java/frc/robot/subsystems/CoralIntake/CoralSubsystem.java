
package frc.robot.subsystems.CoralIntake;

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

public class CoralSubsystem extends SubsystemBase {
  SparkMax coralMotor = new SparkMax(MotorConstants.CORAL_INTAKE_CAN_ID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  public CoralSubsystem() {
      //coralMotor.getConfigurator().apply(new TalonFXConfiguration());
      coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    // Pushes new speed to intake wheel motor
  public void setIntakeSpeed(double speed) {
    // Updates intake wheel speed on dashboard
    //ringOut.Output = speed;
    SmartDashboard.putNumber("Intake Speed: ", speed);
    coralMotor.set(speed);
  }
  /*
  public Command deliverCoral() {
    return this.startEnd(() -> this.setIntakeSpeed(0.3), () -> this.setIntakeSpeed(0));
  }
  public Command armToDeliver() {
    return this.startEnd(() -> this.)

  }
  public Command armToPickup() {

  }
  */
}