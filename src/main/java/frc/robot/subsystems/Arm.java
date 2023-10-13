// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  //keep track of current arm position
  public enum ArmPosition{
    STARTING,
    FRONT,
    BACK
  }

  //initialize variables and obejcts
  ArmPosition currentPosition = ArmPosition.FRONT;
  ArmPosition lastPosition;

  //motor declarations
  TalonFX ArmMotor = new TalonFX(canDeviceIds.ARM_MOTOR_PORT);

  // Servo limelightServo = new Servo(ServoPortConstants.LIMELIGHT_SERVO_PORT);
  // public boolean isLimeLightFront = true;

  public Arm() {
    // ArmMotor_slave.follow(ArmMotor);
    ArmMotor.setNeutralMode(NeutralMode.Brake); //arm should be in brake mode
    ArmMotor.setSelectedSensorPosition(0);
  }

  public CommandBase setPosition(int position){ //this method will raise the arm, then shoot out at that posiiton. Arm will fall back to resting state.
    if(position == 1){ //MIDDLE
        return runOnce(() -> {
                if(ArmMotor.getSelectedSensorPosition() > 0){ //if front side
                    while(ArmMotor.getSelectedSensorPosition() > EncoderConstants.FRONT_MIDDLE_COUNT - 100){
                        ArmMotor.set(ControlMode.PercentOutput, -.3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.53, 2);
                }else{ //if back side
                    while(ArmMotor.getSelectedSensorPosition() < EncoderConstants.BACK_MIDDLE_COUNT + 100){
                        ArmMotor.set(ControlMode.PercentOutput, .3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.40, 2);
                }
            }
        );
    } else if(position == 2){ //HIGH
        return runOnce(() -> {
            if(ArmMotor.getSelectedSensorPosition() > 0){ //if front side
                    while(ArmMotor.getSelectedSensorPosition() > EncoderConstants.FRONT_TOP_COUNT - 100){
                        ArmMotor.set(ControlMode.PercentOutput, -.3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.8, 3);
                }else{ //if back side
                    while(ArmMotor.getSelectedSensorPosition() < EncoderConstants.BACK_TOP_COUNT + 100){
                        ArmMotor.set(ControlMode.PercentOutput, .4);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.8, 3);
                }
            }
        );
    } else if(position == 5){ //Low + stop intake
      return runOnce(() -> {
        RobotContainer.m_intake.dispense(.5, 3.5);
        RobotContainer.m_intake.stopIntake();
        });
    } else{ //LOW
        return runOnce(() -> { //doesn't matter if front or back side.
           RobotContainer.m_intake.dispense(.5, 3.5);
         });
    } 
  }

  public CommandBase flip(){
    return runOnce(
        () ->{
            if(ArmMotor.getSelectedSensorPosition() > 0){ //if front side
                while(ArmMotor.getSelectedSensorPosition() > -1000){
                    ArmMotor.set(ControlMode.PercentOutput, -.4); //move the arm backward
                }
                ArmMotor.set(ControlMode.PercentOutput, 0);
            }else{
                while(ArmMotor.getSelectedSensorPosition() <  1000){ //if back side
                    ArmMotor.set(ControlMode.PercentOutput, .4); //move arm forward
                }
                ArmMotor.set(ControlMode.PercentOutput, 0);
            }
            //these all need to flip with arm
            // flipLimeServo(); //direction of limelight
            RobotContainer.m_Vision.changePipeline(); //limelight pipeline
            if(currentPosition == ArmPosition.FRONT){ //flip arm position enum
                currentPosition = ArmPosition.BACK;
            }else{
                currentPosition = ArmPosition.FRONT;
            }
        }
    );
  }

  public boolean parallelFlip(){ //TESTING THIS APPRAOCH ~ hopefully will solve issues of commands not running in parallel
    if(currentPosition == ArmPosition.FRONT){ //if the arm is in front
        if(ArmMotor.getSelectedSensorPosition() > -1000){ //while not over the middle
            ArmMotor.set(ControlMode.PercentOutput, -.4); //set the speed to move back
            return false;
        }else{
            parallelFlipEnd(); //at the end of the flip, flip the limelight, pipeline, and direction
            ArmMotor.set(ControlMode.PercentOutput, 0); //set the speed to move back
            return true;
        }
    }else{
        if(ArmMotor.getSelectedSensorPosition() < 1000){ //if not over middle yet
            ArmMotor.set(ControlMode.PercentOutput, .4); //move arm toward front
            return false;
        }else{
            ArmMotor.set(ControlMode.PercentOutput, 0); //set the speed to move back
            parallelFlipEnd(); //when over the middle, flip all that need to be flipped
            return true;
        }
    }
  }

  public void parallelFlipEnd(){ //flip all values that need to be flipped after the arm flips
    // flipLimeServo();
    RobotContainer.m_Vision.changePipeline();
    if(currentPosition == ArmPosition.FRONT){
        currentPosition = ArmPosition.BACK;
    }else{
        currentPosition = ArmPosition.FRONT;
    }
  }

  public CommandBase autoNudge(){ //nudges the arm forward for the beginning of a match
    return runOnce(()->{ //move the arm to the down position in the back (non-battery side)?
        while(ArmMotor.getSelectedSensorPosition() > EncoderConstants.BACK_TOP_COUNT + 10000){
            ArmMotor.set(ControlMode.PercentOutput, -.15);
        }
        ArmMotor.set(ControlMode.PercentOutput, 0);
        currentPosition = ArmPosition.BACK;
    });
  }

  public CommandBase waitForArm(){ //do nothing until the arm is at the bottom, only used in auto
    return runOnce(()->{
        while(Math.abs(ArmMotor.getSelectedSensorPosition()) < 20000){
            //do nothing
        }
    });
  }

  public CommandBase autoNudgeThreeCube(){ //nudges the arm forward for the beginning of a match
    return runOnce(()->{ //move the arm to the down position in the back (non-battery side)?
        while(ArmMotor.getSelectedSensorPosition() > EncoderConstants.BACK_TOP_COUNT + 7500){
            ArmMotor.set(ControlMode.PercentOutput, -.22);
        }
        ArmMotor.set(ControlMode.PercentOutput, 0);
        currentPosition = ArmPosition.BACK;
    });
  }

  public CommandBase waitForArmThreeCube(){ //do nothing until the arm is at the bottom, only used in auto
    return runOnce(()->{
        while(Math.abs(ArmMotor.getSelectedSensorPosition()) < Math.abs(EncoderConstants.BACK_BOTTOM_COUNT + 3000)){
            //do nothing
        }
        

    });
  }

  public void setArmPosition(double position) { //not used
    //DO NOTHING
    ArmMotor.set(TalonFXControlMode.Position, position);
  }

  public int getDirection(){ //hardly used
    if(currentPosition == ArmPosition.STARTING){
        return 99;
    }else if(currentPosition == ArmPosition.FRONT){
        return 1;
    }else{
        return -1;
    }
  }

  public double getEncoderPosition(){ //used for testing
    return ArmMotor.getSelectedSensorPosition();
  }

  public void zeroEncoder(){ //initialize
    ArmMotor.setSelectedSensorPosition(0);
  }
  public CommandBase resetEncoder(){ //can reset encoder that has strayed away from required vals. Useful if motor loses accuracy
    return runOnce(()->{
        if(ArmMotor.getSelectedSensorPosition() < 0){
            ArmMotor.setSelectedSensorPosition(EncoderConstants.BACK_BOTTOM_COUNT);
        }else{
            ArmMotor.setSelectedSensorPosition(EncoderConstants.FRONT_BOTTOM_COUNT);
        }
    });
  }

  public void setFrontBottom(){
    ArmMotor.setSelectedSensorPosition(EncoderConstants.FRONT_BOTTOM_COUNT);
  }

  public void setBackBottom(){
    ArmMotor.setSelectedSensorPosition(EncoderConstants.BACK_BOTTOM_COUNT);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}


  // public double getLimelightAngle(){
  //   return limelightServo.getAngle();
  // }


  // public void flipLimeServo(){ //rotate limelight servo
  //    if(isLimeLightFront){ //battery side
  //        limelightServo.setAngle(178);
    //   }else{ //no-battery side
    //       limelightServo.setAngle(-5);
    //   }
    //   isLimeLightFront = !isLimeLightFront; //flip the direction of the limelight
      
    // }

  }
