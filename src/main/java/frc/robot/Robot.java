// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  Timer disabledTimer;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kThreeCubeLeftAuto = "ThreeCubeLeft";
  private static final String kThreeCubeRightAuto = "ThreeCubeRight";
  private static final String kTwoCubeAuto = "TwoCube";
  private static final String kOneCubeDrive = "OneCubeDrive";
  private static final String kOneCubeNoDrive = "OneCubeNoDrive";
  private static final String kDriveOnly = "Drive";
  private static final String kDriveDock= "OneCubeDock";

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    PathPlannerServer.startServer(5811);
    RobotContainer.m_Arm.resetEncoder();
    RobotContainer.m_Drivetrain.resetGyro();
    disabledTimer = new Timer();
    RobotContainer.m_Drivetrain.resetOdometry(new Pose2d());


    m_chooser.setDefaultOption("Two Cube", kTwoCubeAuto);
    m_chooser.addOption("kTest", "kTest");
    m_chooser.addOption("Three Cube Left", kThreeCubeLeftAuto);
    m_chooser.addOption("Three Cube Right", kThreeCubeRightAuto);
    m_chooser.addOption("One Cube Drive", kOneCubeDrive);
    m_chooser.addOption("One Cube *no* Drive", kOneCubeNoDrive);
    m_chooser.addOption("Only Drive", kDriveOnly);
    m_chooser.addOption("One Cube dock", kDriveDock);
    SmartDashboard.putData("Auto Choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    RobotContainer.m_Arm.zeroEncoder();
    RobotContainer.m_Drivetrain.setBrakeMode();
    disabledTimer.reset();
    disabledTimer.start();

  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.m_Arm.zeroEncoder();
    if (disabledTimer.hasElapsed(3))
    {
      RobotContainer.m_Drivetrain.setCoastMode();
      disabledTimer.stop();
    }
  }

  @Override
  public void autonomousInit() {
    RobotContainer.m_Drivetrain.initializeEncoders();
    RobotContainer.m_Vision.setToFrontPipeline();
    RobotContainer.m_Drivetrain.setBrakeMode();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected());
    System.out.println("Selected Auto: " + m_chooser.getSelected());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    RobotContainer.m_Drivetrain.resetOdometry(new Pose2d());

    RobotContainer.m_Drivetrain.initializeEncoders();
    RobotContainer.m_Drivetrain.setBrakeMode();

    if(m_chooser.getSelected().equals("TwoCube")){
      RobotContainer.m_Arm.setBackBottom();
    }else if(m_chooser.getSelected().equals("OneCubeDrive")){
      RobotContainer.m_Arm.setFrontBottom();
    }else if(m_chooser.getSelected().equals("ThreeCubeLeft")){
        RobotContainer.m_Arm.setBackBottom();
    }else if(m_chooser.getSelected().equals("ThreeCubeRight")){
          RobotContainer.m_Arm.setBackBottom();
    } else if(m_chooser.getSelected().equals("OneCubeNoDrive")){
      RobotContainer.m_Arm.setBackBottom();
    } else if(m_chooser.getSelected().equals("OneCubeDock")){
        RobotContainer.m_Arm.setFrontBottom();
    } else{
      RobotContainer.m_Arm.setBackBottom();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}
  

  @Override
  public void testInit(){
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();

    RobotContainer.m_Arm.zeroEncoder();
  } 

  @Override
  public void testPeriodic() {
   
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
