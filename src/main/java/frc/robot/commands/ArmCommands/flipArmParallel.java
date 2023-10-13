// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class flipArmParallel extends CommandBase {
  boolean done = false;
  public flipArmParallel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Arm);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    done = RobotContainer.m_Arm.parallelFlip();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return done;
  }
}