// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveDistPid extends CommandBase {
  double angle;
  public PIDController pidTurn;
  public PIDController pidDist;
  double in;
  double EncoderCountDist;

  /** Creates a new TurnAngle. */
  public DriveDistPid(double inches, double ang, double spd, double kp, double ki, double kd) {
    // Use a%ddRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
    angle = ang;
    pidTurn = new PIDController(kp, ki, kd); //TUNE 
    pidDist = new PIDController(0.00001, 0, 0);
    in = inches;
    EncoderCountDist = -1*inches*2048*3/8/1.8;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Drivetrain.resetLeftEncoder();
    RobotContainer.m_Drivetrain.resetRightEncoder();
    pidTurn.setSetpoint(angle);
    pidDist.setTolerance(1); //TUNE
    pidDist.setSetpoint(in);
    pidDist.reset();
    RobotContainer.m_Drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_command = -1 * pidTurn.calculate(RobotContainer.m_Drivetrain.getRobotYaw());
    double right_command = pidTurn.calculate(RobotContainer.m_Drivetrain.getRobotYaw());
    
    double leftSpeed = pidDist.calculate(RobotContainer.m_Drivetrain.getLeftEncoder());
    
    System.out.println("Left: " + leftSpeed);
    // System.out.println("Right: " + );
    System.out.println("Error: " + pidDist.getPositionError());

    RobotContainer.m_Drivetrain.DriveTank((leftSpeed + left_command), (leftSpeed + right_command) * -1 * 0.90);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidDist.atSetpoint();
  }
}
