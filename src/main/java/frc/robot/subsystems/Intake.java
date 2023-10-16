// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
// import frc.robot.subsystems.Arm.ArmPosition;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //Spark Max setup
  CANSparkMax intake1 = new CANSparkMax(motorPortConstants.INTAKE_PORT_1, MotorType.kBrushless);
  CANSparkMax intake2 = new CANSparkMax(motorPortConstants.INTAKE_PORT_2, MotorType.kBrushless);
  //Limit Switch
  DigitalInput intakeLimitSwitch = new DigitalInput(0); //LIMIT SWITCH
  RelativeEncoder intakeEncoder = intake1.getEncoder();
  // //Pneumatic Hub/Compressor Setup
  // PneumaticHub phHub = new PneumaticHub(pneumaticConstants.REV_HUB_PORT);
  // Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  // //Double Solenoid Setup (will experiment with the "DoubleSolenoid" class later)
  // Solenoid m_solenoid = phHub.makeSolenoid(pneumaticConstants.SOLENOID_PORT);


  public boolean holding = false;
 
  public Intake() {
    intake1.setInverted(true);
    intake2.setInverted(true);

    intake1.setSmartCurrentLimit(25);
    intake1.setSmartCurrentLimit(25);
    

  }

  public void run_in(){
    if(intakeLimitSwitch.get() == true){ //if limit switch is not held
      holding = false; //we are not holding a cube
      intake1.set(0.35); //set intaking speeds
      intake2.set(-0.35);
      // setSolenoidState(holding);
      
    }else{
      if(!holding){ //rumble not working (no rumble motor in our controller)
        // RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1); 
        // RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1);
      }
      intake1.set(0); //stop the intake
      intake2.set(0);
      holding = true;
      // setSolenoidState(holding);
    }
  }

  public void run_out(){ //outtake at full speed
    intake1.set(-1);
    intake2.set(1);
  }
  public void stop(){ //stop the intake
    intake1.set(0);
    intake2.set(0);
  }

  public void stopIntake() {
      intake1.set(0);
      intake2.set(0);
  }

  public CommandBase startIntake() {
    return runOnce( () -> {
      intake1.set(0.4);
      intake2.set(-0.4);
    });
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
  
  // public void setSolenoidState(Boolean state){
  //   m_solenoid.set(state);
  //   System.out.println("Firing Piston, "  + state);
  // }

  
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
