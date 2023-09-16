// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static class EncoderConstants{
    public static int STARTING_COUNT = 0;

    // public static final int FRONT_TOP_COUNT = 13500;
    // public static final int FRONT_MIDDLE_COUNT = 17000;
    public static final int FRONT_TOP_COUNT = 12000;
    public static final int FRONT_MIDDLE_COUNT = 15000;

    public static final int FRONT_BOTTOM_COUNT = 24450; //adjust for zeroing
    //arm rotation encoder counts
    public static final int BACK_BOTTOM_COUNT = -26100; //adjust for zeroing
    public static final int BACK_MIDDLE_COUNT = -17000;
    public static final int BACK_TOP_COUNT = -17000;
    //telescoping arm encoder counts
    public static final int ARM_EXTENDED_COUNT = 0;
    public static final int ARM_UNEXTENDED_COUNT = 0;
  }
  //intake speed counts
  public static final int INTAKE_SPEED = 0;
  public static final int OUTTAKE_SPEED = 0;
  
  public static class PathPlanConstants {
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kTrackwidthMeters = 0.69;
    public final static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public class PidConstants {
    public static final double DRIVE_SPEED = 0.5;
    public static final double TURN_SPEED = 0.5;
    public static final double kp_DRIVE = 0.003;
    public static final double ki_DRIVE = 0.0004; 
    public static final double kd_DRIVE = 0.001;
    public static final double kp_TURN = 0.03;
    public static final double ki_TURN = 0.00;
    public static final double kd_TURN = 0.0035;
    
    public static final double TURN_SPEEDRIGHT = 0.3;
    public static final double kp_DRIVERIGHT = 0.003;
    public static final double ki_DRIVERIGHT = 0.0008; 
    public static final double kd_DRIVERIGHT = 0.001;
    public static final double kp_TURNRIGHT = 0.0085;
    public static final double ki_TURNRIGHT = 0.00;
    public static final double kd_TURNRIGHT = 0.0035;
  }

  public class motorPortConstants{
    //MOTOR PORTS
    //DRIVE_TRAIN
    public static final int  DRIVE_TRAIN_RIGHT_1 = 1;
    public static final int DRIVE_TRAIN_RIGHT_2 = 2;
    public static final int DRIVE_TRAIN_LEFT_1 = 3;
    public static final int DRIVE_TRAIN_LEFT_2 = 4;
    //ARM
    public static final int ARM_MOTOR_PORT = 10;
    public static final int ARM_MOTOR_SLAVE_PORT = 9;//This motor is not on the robot right now.
    public static final int ARM_EXTEND_PORT = 7;
    //INTAKE
    public static final int INTAKE_PORT_1 = 5;
    public static final int INTAKE_PORT_2 = 6;
  }
  public class ServoPortConstants{
    //SERVOS
    public static final int LIMELIGHT_SERVO_PORT = 3;
  }

  public class limeLightConstants{
    public static final double KP = 0.1;
    public static final double MIN_COMMAND = 0.05;
  }

}
