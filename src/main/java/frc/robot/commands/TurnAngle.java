// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnAngle extends CommandBase {
  double ang;
  public PIDController pid;
  boolean done;

  /** Creates a new TurnAngle. */
  public TurnAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
    ang = angle;
    pid = new PIDController(0.005, 0, 0.00); //TUNE 
    done = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Drivetrain.resetGyro();
    pid.enableContinuousInput(-180.0f,  180.0f); //TUNE
    pid.setTolerance(1, 0); //TUNE
    pid.setSetpoint(ang);
    pid.reset();
    

    // pid.setP(0.0055);
    // pid.setI(0.0013);
    // pid.setD(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Turn Command Called " + ang);
    double left_command = pid.calculate(RobotContainer.m_Drivetrain.getRobotYaw());
    double right_command = pid.calculate(RobotContainer.m_Drivetrain.getRobotYaw());

    
    RobotContainer.m_Drivetrain.left1.set(ControlMode.PercentOutput, -left_command);
    RobotContainer.m_Drivetrain.left2.set(ControlMode.PercentOutput, -left_command);
    RobotContainer.m_Drivetrain.right1.set(ControlMode.PercentOutput, -right_command);
    RobotContainer.m_Drivetrain.right2.set(ControlMode.PercentOutput, -right_command);
    //RobotContainer.m_Drivetrain.DriveTank(left_command * -1, right_command * -1);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
