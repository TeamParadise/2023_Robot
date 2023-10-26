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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.LEDCommands.ShowSide;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //Spark Max setup
  CANSparkMax intake1 = new CANSparkMax(canDeviceIds.INTAKE_PORT_1, MotorType.kBrushless);
  CANSparkMax intake2 = new CANSparkMax(canDeviceIds.INTAKE_PORT_2, MotorType.kBrushless);
  //Limit Switch
  DigitalInput intakeLimitSwitch = new DigitalInput(0); //LIMIT SWITCH
  RelativeEncoder intakeEncoder = intake1.getEncoder();

  public int not_holding = 5;
 
  public Intake() {
    intake1.setInverted(true);
    intake2.setInverted(true);
  }

  public void run_in(){
    if(intakeLimitSwitch.get() == true){ //if limit switch is not held
      not_holding = 0; //we are not holding a cube
      intake1.set(.4); //set intaking speeds
      intake2.set(-.4);
      
    }else{
      stopIntake();
      if (not_holding < 5) {
        System.out.println("Running");
        RobotContainer.m_LED.blink(5);
        /* Commands.run(new ShowCube(0.3)), null);
          RobotContainer.m_LED.setColor(0.99);
          System.out.println("Black");
          new WaitCommand(0.3);
          RobotContainer.m_LED.setColor(ShowSide.LedColor);
          System.out.println("Color");
          new WaitCommand(0.3); */
      }
      not_holding = 5;
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
