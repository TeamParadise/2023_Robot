// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetColor;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.flipLimelight;
import frc.robot.commands.ArmCommands.flipArmParallel;
import frc.robot.commands.AutoCommands.OneCubeDock;
import frc.robot.commands.AutoCommands.OneCubeDrive;
import frc.robot.commands.AutoCommands.OneCubeNoDrive;
import frc.robot.commands.AutoCommands.OnlyDrive;
import frc.robot.commands.AutoCommands.ThreeCubeLeft;
import frc.robot.commands.AutoCommands.ThreeCubeRight;
import frc.robot.commands.AutoCommands.Turn45;
import frc.robot.commands.AutoCommands.TwoCube;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static Drivetrain m_Drivetrain = new Drivetrain();
  public final static Arm m_Arm = new Arm();
  public static Intake m_intake = new Intake();
  public static Vision m_Vision = new Vision();
  public static LedSubsystem m_led = new LedSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
   public static  XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);
   public static  XboxController coDriverController = new XboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings(
      // private final CommandXboxController Intake = 
    );
    m_Drivetrain.setDefaultCommand(new DriveArcade());
    m_intake.setDefaultCommand(new IntakeCommand());
    m_led.setDefaultCommand(new SetColor(-0.99));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    new JoystickButton(driverController, 1).onTrue(m_Arm.setPosition(5)); //a *Low Goal
    new JoystickButton(driverController, 2).onTrue(m_Arm.setPosition(1)); //b *Middle Goal
    new JoystickButton(driverController, 3).onTrue(m_Arm.setPosition(2)); //x *High Goal
    new JoystickButton(driverController, 4).onTrue(new flipArmParallel()/*.withTimeout(7.5)*/); //y *flip arm

    new JoystickButton(driverController, 11).onTrue(new TurnToTarget()); //left bumper *Limelight
    // new JoystickButton(driverController, 2).onTrue(m_Drivetrain.resetGyro()); //b

    // new JoystickButton(driverController, 6).onTrue(new DriveDist()); //right bumper

    new JoystickButton(driverController, 12).onTrue(m_Drivetrain.invertDrive()); //RB *flip drive

    // new JoystickButton(driverController,3).onTrue(new TurnAngle(45)); //x
    // new JoystickButton(driverController, 5).onTrue(new DriveDist(0, -PidConstants.DRIVE_SPEED, PidConstants.kp_DRIVE, PidConstants.ki_DRIVE, PidConstants.kd_DRIVE)); //left bumper


    new JoystickButton(coDriverController, 1).onTrue(m_Drivetrain.halfSpeed()); //Co Drive A //Speed in Half
    new JoystickButton(coDriverController, 2).onTrue(m_Drivetrain.toggleBrake()); //Co Drive B //Toggle brake mode
    new JoystickButton(coDriverController, 3).onTrue(m_Arm.resetEncoder()); //Co Drive X //Reset Arm position
    new JoystickButton(coDriverController, 4).onTrue(m_Arm.toggleArmPosition()); //Co Drive Y //Toggle Arm Position

    // new JoystickButton(coDriverController, 5).onTrue(new changePipeline()); //left bumper
    new JoystickButton(coDriverController, 5).onTrue(m_intake.startIntake()); //left bumper
    new JoystickButton(coDriverController, 6).onTrue(new flipLimelight()); //right bumper
    new JoystickButton(driverController, 5).onTrue(new SetColor(0.85)); //Left Trigger
    new JoystickButton(driverController, 6).onTrue(new SetColor(0.62)); //Right Trigger
    

    // new POVButton(driverController, ).onTrue(m_Arm.setPosition(1));
    // new Trigger(driverController.povUp(null)).onTrue(m_Arm.setPosition(2));
    // new Trigger(driverController.povLeft(null)).onTrue(m_Arm.flip());

    // driverController.povLeft(null).onTrue(m_Arm.setPosition(1));
    // driverController.povLeft(null).onTrue(m_Arm.setPosition(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String auto) {
    // An example command will be run in autonomous
    if(auto.equals("TwoCube")){
      return new TwoCube();
    } else if (auto.equals("ThreeCubeLeft")){
      return new ThreeCubeLeft();
    } else if (auto.equals("ThreeCubeRight")){
      return new ThreeCubeRight();
    } else if (auto.equals("Turn45")){
      return new Turn45();
    }else if(auto.equals("OneCubeDrive")){
      return new OneCubeDrive();
    }else if(auto.equals("OneCubeNoDrive")){
      return new OneCubeNoDrive();
    }else if(auto.equals("OneCubeDock")){
      return new OneCubeDock();
    }else{
      return new OnlyDrive();
    }
  }
}
