
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands.IntakeCommand;
import frc.robot.commands.ArmCommands.flipArmParallel;

import frc.robot.commands.AutoCommands.OnlyDrive;
import frc.robot.commands.AutoCommands.TwoCube;
import frc.robot.commands.AutoCommands.BackupRoutines.OneCubeDrive;
import frc.robot.commands.AutoCommands.BackupRoutines.OneCubeNoDrive;
import frc.robot.commands.AutoCommands.BackupRoutines.ThreeCubeLeft;
import frc.robot.commands.AutoCommands.BackupRoutines.ThreeCubeRight;
import frc.robot.commands.DriveCommands.DriveArcade;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static Drivetrain m_Drivetrain = new Drivetrain();
  public final static Arm m_Arm = new Arm();
  public static Intake m_intake = new Intake();
  public static Vision m_Vision = new Vision();

  public static  CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static  CommandXboxController coDriverController = new CommandXboxController(OperatorConstants.kCoDriverControllerPort);
  public static XboxController xDriverController = new XboxController(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    m_Drivetrain.setDefaultCommand(new DriveArcade());
    m_intake.setDefaultCommand(new IntakeCommand());
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driverController.a().onTrue(m_Arm.setPosition(3)); //a *Low Goal
    driverController.b().onTrue(m_Arm.setPosition(1)); //b *Middle Goal
    driverController.x().onTrue(m_Arm.setPosition(2)); //x *High Goal
    driverController.y().onTrue(new flipArmParallel()); //y *flip arm
    driverController.rightBumper().onTrue(m_Drivetrain.invertDrive()); //RB *flip drived
    
    coDriverController.a().onTrue(m_Drivetrain.halfSpeed()); //Co Drive A //Speed in Half
    coDriverController.b().onTrue(m_Drivetrain.toggleBrake()); //Co Drive B //Toggle brake mode
    coDriverController.x().onTrue(new DriveArcade()); //Co Drive X //Reset Arm position
    coDriverController.leftBumper().onTrue(m_intake.startIntake()); //left bumper
    coDriverController.leftTrigger().onTrue(m_Drivetrain.resetGyro()); //left trigger
  }

  public Command getAutonomousCommand(String auto) {
    if (auto.equals("kTest")) return RobotContainer.m_Drivetrain.followPath("Test");
    else if(auto.equals("TwoCube")) return new TwoCube();
    else if (auto.equals("ThreeCubeLeft")) return new ThreeCubeLeft();
    else if (auto.equals("ThreeCubeRight")) return new ThreeCubeRight();
    else if(auto.equals("OneCubeDrive")) return new OneCubeDrive();
    else if(auto.equals("OneCubeNoDrive")) return new OneCubeNoDrive();
    // else if(auto.equals("OneCubeDock"))return new OneCubeDock();
    else return new OnlyDrive();
  }
}