// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//limelight imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public boolean isFront = true;
  /** Creates a new ExampleSubsystem. */
  public Vision() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); //front pipeline
  }

  public void setToFrontPipeline(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); //front pipeline
  }

  public CommandBase setToBackPipeline(){
    return runOnce(() ->{
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //front pipeline
    });
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

  public void printVals(){
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    System.out.println("x: " +  x+"\ny: " + y);
  }

  double steering_adjust = 0;
  double leftSpeed = 0;
  double rightSpeed = 0;

  public boolean targetTurn(Drivetrain d){
      System.out.println("TURN TO TARGET");
      double x = tx.getDouble(0.0);
      if(Math.abs(x) > .7){
        if(x < 0){
          d.arcadeDrive(-.35, 0);
        }else if(x > 0){
          d.arcadeDrive(.35, 0);
        }
        return false;
      }else{
        d.arcadeDrive(0, 0);
        return true;

      }
  }
  public boolean targetTurnScaled(Drivetrain d){
    System.out.println("TURN TO TARGET! isFront? " +isFront);
    double x = tx.getDouble(0.0);
    double y = tx.getDouble(0);
    if(isFront){ //Battery side camera
      double kp = -0.05;
      double min_command = 0.25;
  
      double heading_error = -1*x;
      double left_command = 0;
      double right_command = 0;
      if(Math.abs(heading_error) > 1){
        if(heading_error< 0){
          steering_adjust = kp*heading_error + min_command;
        }else{
          steering_adjust = kp*heading_error - min_command;
        }
        left_command += steering_adjust;
        right_command -= steering_adjust;
        if(left_command > .4){
          left_command = .4;
        }else if(left_command < -.4){
          left_command = -.4;
        }

        if(right_command > .4){
          right_command = .4;
        }else if(right_command < -.4){
          right_command = -.4;
        }
        System.out.println("left: " + left_command + " right: " + right_command + isFront);
        d.tankDrive(left_command, -right_command);
        return false;
      }else{
        d.arcadeDrive(0, 0);
        return true;
  
      }
    }else{ //Non-Battery Side
      double kp = -0.03;
      double min_command = 0.25;
  
      double heading_error = -1*x;
      double left_command = 0;
      double right_command = 0;
      if(Math.abs(heading_error) > 1){
        RobotContainer.m_Drivetrain.setBrakeMode();
        if(heading_error< 0){
          steering_adjust = kp*heading_error + min_command;
        }else{
          steering_adjust = kp*heading_error - min_command;
        }
        left_command += steering_adjust;
        right_command -= steering_adjust;
        if(left_command > .4){
          left_command = .4;
        }else if(left_command < -.4){
          left_command = -.4;
        }

        if(right_command > .4){
          right_command = .4;
        }else if(right_command < -.4){
          right_command = -.4;
        }
        System.out.println("left: " + left_command + " right: " + right_command + isFront);
        d.tankDrive(left_command, -right_command);
        return false;
      }else{
        d.arcadeDrive(0, 0);
        RobotContainer.m_Drivetrain.setCoastMode();
        return true;
  
      }
    }
    
  }

  public boolean isFinished(){
    return (Math.abs(tx.getDouble(0)) < .7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean changePipeline(){
    if(isFront){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //back pipeline
      System.out.println("SWITCHING TO NON-BATTERY SIDE PIPELINE");
    }else{
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); //front pipeline
      System.out.println("SWITCHING TO **BATTERY** SIDE PIPELINE");
    }
    isFront = !isFront; //switch isFront var
    System.out.println(isFront);
    return true;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
