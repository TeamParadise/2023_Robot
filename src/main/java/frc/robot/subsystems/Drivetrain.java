// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;



public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_TalonFX right1 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_RIGHT_1);
  WPI_TalonFX right2 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_RIGHT_2);

  WPI_TalonFX left1 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_LEFT_1);
  WPI_TalonFX left2 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_LEFT_2);


  MotorControllerGroup leftMotors = new MotorControllerGroup(left1, left2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(right1, right2 );

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  
  boolean invertedDrive = false;

  public Drivetrain() {
    
  }

  public CommandBase invertDrive(){
    return runOnce(()->{
      invertedDrive = !invertedDrive;
    });
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    if(!invertedDrive){
      drive.arcadeDrive(moveSpeed, rotateSpeed);

    }else{
      drive.arcadeDrive(moveSpeed, -1*rotateSpeed);
    }
  }

  public void tankDrive(double left, double right){
    drive.tankDrive(left, right);
  }

  double leftSpeed = .40;
  double rightSpeed = .40;
  public boolean driveDistance(int inches){

    //2048*6/8 = 18"
    //2048*12/6 = 
    double EncoderCountDist = -1*inches*2048*6/8/1.8; 
    // System.out.println("DRIVE DISTANCE");
    // System.out.println("CURRENT: " + left1.getSelectedSensorPosition() + " Goal: " + EncoderCountDist);
      
      // if(left1.getSelectedSensorPosition() > EncoderCountDist){
      //   drive.tankDrive(-.4, .4);
      //   return false;
      // }else{
      //   drive.tankDrive(0, 0);
      //   return true;
      // }

      if(left1.getSelectedSensorPosition() > EncoderCountDist){
        if(-1*left1.getSelectedSensorPosition() > right1.getSelectedSensorPosition()){
          rightSpeed += .0015;
        }else if(-1*left1.getSelectedSensorPosition() < right1.getSelectedSensorPosition()){
          rightSpeed -= .0015;
        }else{
          //do nothing
        }
        drive.tankDrive(-leftSpeed, rightSpeed);
        return false;
      }else{
        drive.tankDrive(0, 0);
        initializeEncoders();
        return true;
      }
  }
  boolean firstTimeback = true;
  public boolean driveDistanceBack(int inches){
    if(firstTimeback){
      leftSpeed = .40;
      rightSpeed = .40;
      firstTimeback = false;
    }
    //2048*6/8 = 18"
    //2048*12/6 = 
    double EncoderCountDist = inches*2048*6/8/1.8; 
    System.out.println("DRIVE DISTANCE");
    System.out.println("CURRENT: " + left1.getSelectedSensorPosition() + " Goal: " + EncoderCountDist);
      
      // if(left1.getSelectedSensorPosition() > EncoderCountDist){
      //   drive.tankDrive(-.4, .4);
      //   return false;
      // }else{
      //   drive.tankDrive(0, 0);
      //   return true;
      // }

      if(left1.getSelectedSensorPosition() < EncoderCountDist){
        if(-1*left1.getSelectedSensorPosition() < right1.getSelectedSensorPosition()){
          rightSpeed += .0015;
        }else if(-1*left1.getSelectedSensorPosition() > right1.getSelectedSensorPosition()){
          rightSpeed -= .0015;
        }else{
          //do nothing
        }
        drive.tankDrive(leftSpeed, -rightSpeed);
        return false;
      }else{
        drive.tankDrive(0, 0);
        return true;
      }
  }

  public void initializeEncoders(){
    left1.setSelectedSensorPosition(0);
    left2.setSelectedSensorPosition(0);
    right1.setSelectedSensorPosition(0);
    right2.setSelectedSensorPosition(0);
  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void setBrakeMode(){
    left1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode(){
    left1.setNeutralMode(NeutralMode.Coast);
    left2.setNeutralMode(NeutralMode.Coast);
    right1.setNeutralMode(NeutralMode.Coast);
    right2.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
