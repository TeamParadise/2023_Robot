// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //Spark Max setup
  CANSparkMax intake1 = new CANSparkMax(canDeviceIds.INTAKE_PORT_1, MotorType.kBrushless);
  CANSparkMax intake2 = new CANSparkMax(canDeviceIds.INTAKE_PORT_2, MotorType.kBrushless);
  //Limit Switch
  DigitalInput intakeLimitSwitch = new DigitalInput(0); //LIMIT SWITCH
  RelativeEncoder intakeEncoder = intake1.getEncoder();

  public boolean not_holding = true;
 
  public Intake() {
    intake1.setInverted(true);
    intake2.setInverted(true);
  }

  public void run_in(){

    if(intakeLimitSwitch.get() == true){ //if limit switch is not held
      not_holding = true; //we are not holding a cube
      intake1.set(.4); //set intaking speeds
      intake2.set(-.4);
      
    }else{
      stopIntake();
      not_holding = false;
    }
  }

  public CommandBase startIntake() {
    return runOnce( () -> {
      intake1.set(0.4);
      intake2.set(-0.4);
    });
  }
  
  public void stopIntake() {
    intake1.set(0);
    intake2.set(0);
  }

  public void dispense(double speed, double rotations){ //dispense at a speed and certain number of rotations
    System.out.println("DISPENSE!!!!");
    intakeEncoder.setPosition(0);
    while(intakeEncoder.getPosition() > rotations*-8){
      intake1.set(-1*speed);
      intake2.set(speed);
    }
  }

  public double getEncoderCount(){ 
    return intakeEncoder.getPosition();
  }

  public void resetEncoder(){
    intakeEncoder.setPosition(0);
  }
}
