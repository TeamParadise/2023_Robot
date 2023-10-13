// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.BackupRoutines;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class OneCubeNoDrive extends SequentialCommandGroup {
  

  public OneCubeNoDrive() {
    addCommands(
      RobotContainer.m_Arm.autoNudge(),
      RobotContainer.m_Arm.waitForArm(),
      RobotContainer.m_Arm.setPosition(2)
      // new ParallelCommandGroup(new DriveDist(), RobotContainer.m_Arm.flip()),
      // new ParallelCommandGroup(new DriveDistBack(), RobotContainer.m_Arm.flip()),
      // RobotContainer.m_Vision.setToBackPipeline(),     
      // new TurnToTarget(),
      // RobotContainer.m_Arm.setPosition(3)
    );
  }
}
