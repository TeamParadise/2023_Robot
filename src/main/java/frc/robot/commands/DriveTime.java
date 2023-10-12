// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTime extends CommandBase {
  double speed;
  double angle;
  public PIDController pid;
  double in;
  double EncoderCountDist;

  /** Creates a new TurnAngle. */
  public DriveTime(double ang, double spd, double kp, double ki, double kd) {
    // Use a%ddRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
    // RobotContainer.m_Drivetrain.resetGyro();
    speed = spd;
    angle = ang;
    pid = new PIDController(kp, ki, kd); //TUNE 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Drivetrain.resetGyro();
    pid.enableContinuousInput(-180.0f,  180.0f); //TUNE
    pid.setTolerance(0, 0.1); //TUNE
    pid.setSetpoint(angle);
    pid.reset();
    RobotContainer.m_Drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_command = -1 * pid.calculate(RobotContainer.m_Drivetrain.getRobotAngle());
    double right_command = pid.calculate(RobotContainer.m_Drivetrain.getRobotAngle());
    System.out.println("Left: " + left_command);
    System.out.println("Right: " + right_command);
    System.out.println("Error: " + pid.getPositionError());

    RobotContainer.m_Drivetrain.DriveTank((speed + left_command), (speed + right_command) * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
