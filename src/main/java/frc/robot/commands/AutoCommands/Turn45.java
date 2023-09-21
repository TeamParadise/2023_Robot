// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Turn45 extends SequentialCommandGroup {
  /** Creates a new Turn45. */
  public Turn45() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // RobotContainer.m_Drivetrain.resetGyro(),
      // RobotContainer.m_Arm.autoNudge(),
      // RobotContainer.m_Arm.waitForArmThreeCube()
      new TurnAngle(45).withTimeout(5)
    );
  }
}
