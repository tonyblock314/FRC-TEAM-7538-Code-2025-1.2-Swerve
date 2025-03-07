// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class MotorConstants {
    public static final int CLIMBER_CAN_ID_LEFT = 13;
    public static final int CLIMBER_CAN_ID_RIGHT = 14;
    public static final int ELEVATOR_CAN_ID_LEFT = 15;
    public static final int ELEVATOR_CAN_ID_RIGHT = 16;
    public static final int ALGAE_JORKING_IT_CAN_ID = 17;
    public static final int FONDLING_MOTOR_CAN_ID = 18;
    public static final int CORAL_INTAKE_CAN_ID = 19;
  }

  public static final class ElevatorConstants {
    public static final double ELEVATOR_SCALING_FACTOR = 0.4;
    public static final double LEVEL_1 = 0;
    public static final double LEVEL_2 = 20;
    public static final double LEVEL_3 = 40.5;
    public static final double SOURCE_LEVEL = 15;
    public static final double TOP_LEVEL = 62;


    public static final double kG = 0.1; // Output needed to overcome gravity
    public static final double kS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.01; // An acceleration of 1 rps/s require 0.01 v output
    public static final double kP = 2.4; // An error of 1 rotation results in 2.4 V output
    public static final double kI = 0.01; // no output for integrated error
    public static final double kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    public static final double CRUISEVELOCITY = 60;
    public static final double ACCELERATION = 60;
    public static final double JERK = 0;
  }

  public static final class ArmConstants {
    public static final double ARM_UP_POSITION = 10;
    public static final double ARM_DOWN_POSITION = 0;
    public static final double ARM_INTAKE_POSITION = 5;
    public static final double ARM_INTAKE_GROUND_POSITION = 2;
  }

  public static final class CoralIntakeConstants {
    public static final double CORAL_INTAKE_SPEED_IN = .1;
    public static final double CORAL_INTAKE_SPEED_OUT = .02;
    public static final double kP = 4;
    public static final double kI = 2;
    public static final double kD = .2;
    public static final double kMINOUTPUT = 0;
    public static final double kMAX_OUTPUT = 5;
  }

  public static final class AlgaeIntakeConstants {
    public static final double ALGAE_SCALING_FACTOR = 0.4;
    public static final double AUTO_ALGAE_SPEED = 0.5;
  }

  public static final class ClimberConstants {
    public static final int DOWN_LEVEL = 100;
    public static final int UP_LEVEL = 0;
    public static final double CLIMBER_SCALING_FACTOR = 0.2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.15;
    public static final double LEFT_Y_DEADBAND  = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
    public static final double RIGHT_Y_DEADBAND = 0.15;
    public static final double TURN_CONSTANT    = 6;
    public static final double TRIGGER_DEADBAND = 0.1;
  }
}
