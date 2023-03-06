// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
// import frc.robot.subsystems.Arm.ArmPosition;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax intake1 = new CANSparkMax(motorPortConstants.INTAKE_PORT_1, MotorType.kBrushless);
  CANSparkMax intake2 = new CANSparkMax(motorPortConstants.INTAKE_PORT_2, MotorType.kBrushless);
  DigitalInput intakeLimitSwitch = new DigitalInput(0);

  public Intake() {
    
  }

  public void run_in(){
    if(intakeLimitSwitch.get() == true){
      intake1.set(.5);
      intake2.set(-.5);
    }else{
      intake1.set(0);
      intake2.set(0);
    }
  }

  public void run_out(){
    intake1.set(-1);
    intake2.set(1);
  }
  public void stop(){
    intake1.set(0);
    intake2.set(0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
