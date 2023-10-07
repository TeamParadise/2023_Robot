// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.synth.SynthStyle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.EncoderConstants;
import frc.robot.commands.SetColor;
import frc.robot.subsystems.LedSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
   private static final String kThreeCubeLeftAuto = "ThreeCubeLeft";
   private static final String kThreeCubeRightAuto = "ThreeCubeRight";
   private static final String kTwoCubeAuto = "TwoCube";
   private static final String kTurn45 = "Turn45";
   private static final String kOneCubeDrive = "OneCubeDrive";
   private static final String kOneCubeNoDrive = "OneCubeNoDrive";
   private static final String kDriveOnly = "Drive";
   private static final String kDriveDock= "OneCubeDock";

  //  if(auto.equals("TwoCube")){
  //   return new TwoCube();
  // }else if(auto.equals("OneCubeDrive")){
  //   return new OneCubeDrive();
  // }else if(auto.equals("OneCubeNoDrive")){
  //   return new OneCubeNoDrive();
  // }else{
  //   return new OnlyDrive();
  // }
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    RobotContainer.m_Arm.resetEncoder();
    m_chooser.setDefaultOption("Two Cube", kTwoCubeAuto);
    m_chooser.addOption("Three Cube Left", kThreeCubeLeftAuto);
    m_chooser.addOption("Three Cube Right", kThreeCubeRightAuto);
    m_chooser.addOption("Turn 45", kTurn45);
    m_chooser.addOption("One Cube Drive", kOneCubeDrive);
    m_chooser.addOption("One Cube *no* Drive", kOneCubeNoDrive);
    m_chooser.addOption("Only Drive", kDriveOnly);
    m_chooser.addOption("One Cube dock", kDriveDock);
    SmartDashboard.putData("Auto Choices", m_chooser);

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.m_Arm.zeroEncoder();
    
    RobotContainer.m_Drivetrain.setBrakeMode();

  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.m_Arm.zeroEncoder();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("Selected Auto: " + m_chooser.getSelected());
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected());
    RobotContainer.m_Drivetrain.setBrakeMode();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.m_Drivetrain.initializeEncoders();
    RobotContainer.m_Vision.setToFrontPipeline();
    RobotContainer.m_Drivetrain.setBrakeMode();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    System.out.println("Yaw: " + RobotContainer.m_Drivetrain.getRobotYaw());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.m_Drivetrain.initializeEncoders();
    RobotContainer.m_Drivetrain.setCoastMode();

    if(m_chooser.getSelected().equals("TwoCube")){
      RobotContainer.m_Arm.setFrontBottom();
    }else if(m_chooser.getSelected().equals("OneCubeDrive")){
      RobotContainer.m_Arm.setFrontBottom();
    }else if(m_chooser.getSelected().equals("ThreeCubeLeft")){
        RobotContainer.m_Arm.setBackBottom();
      }else if(m_chooser.getSelected().equals("ThreeCubeRight")){
          RobotContainer.m_Arm.setBackBottom();
    } else if (m_chooser.getSelected().equals("Turn45")){
      RobotContainer.m_Arm.setBackBottom();
    } else if(m_chooser.getSelected().equals("OneCubeNoDrive")){
      RobotContainer.m_Arm.setBackBottom();
    } else if(m_chooser.getSelected().equals("OneCubeDock")){
        RobotContainer.m_Arm.setFrontBottom();
    } else{
      RobotContainer.m_Arm.setBackBottom();
    }

    RobotContainer.m_Drivetrain.brakeModeBool = false;

    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.println("Pitch: " + RobotContainer.m_Drivetrain.getRobotPitch());
    // System.out.println("Yaw: " + RobotContainer.m_Drivetrain.getRobotYaw());
    // System.out.println("Roll: " + RobotContainer.m_Drivetrain.getRobotRoll());
    System.out.println("Arm Position: " + RobotContainer.m_Arm.getEncoderPosition());

    

  }
  

  @Override
  public void testInit(){
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // testArm.setSelectedSensorPosition(0);
    // RobotContainer.m_intake.resetEncoder();
    RobotContainer.m_Arm.zeroEncoder();


  }

 
  @Override
  public void testPeriodic() {
    // RobotContainer.m_Drivetrain.printVals();
    // // System.out.println(RobotContainer.m_Arm.getEncoderPosition());
    // System.out.println("Pitch: " + RobotContainer.m_Drivetrain.getRobotPitch());
    // System.out.println("Yaw: " + RobotContainer.m_Drivetrain.getRobotYaw());
    // System.out.println("Roll: " + RobotContainer.m_Drivetrain.getRobotRoll());
    // System.out.println("Limelgiht angle " + RobotContainer.m_Arm.getLimelightAngle());
    
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
  

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
