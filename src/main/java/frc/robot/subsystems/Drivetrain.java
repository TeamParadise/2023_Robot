// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;



public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonFX right1 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_RIGHT_1);
  public WPI_TalonFX right2 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_RIGHT_2);

  public WPI_TalonFX left1 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_LEFT_1);
  public WPI_TalonFX left2 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_LEFT_2);


  MotorControllerGroup leftMotors = new MotorControllerGroup(left1, left2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(right1, right2);

  double left1PreviousEncoder = left1.getSelectedSensorPosition();
  double right1PreviousEncoder = right1.getSelectedSensorPosition();

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  double leftVelocity = 0;
  double rightVelocity = 0;
  
  boolean invertedDrive = false; //togleable invert front of robot
  boolean halfSpeed = true;
  public boolean brakeModeBool = true;

  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  private final DifferentialDriveOdometry m_odometry;
          
  AHRS ahrs;

  public Drivetrain() {

    ahrs = new AHRS(SerialPort.Port.kUSB);
    // left1.setSafetyEnabled(true);
    m_odometry =
        new DifferentialDriveOdometry(
            ahrs.getRotation2d(), left1.getSelectedSensorPosition(), right1.getSelectedSensorPosition());
      
    right1.configClosedloopRamp(1);
    right2.configClosedloopRamp(1);

    left1.configClosedloopRamp(1);
    left2.configClosedloopRamp(1);

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left1.getSelectedSensorVelocity(), right1.getSelectedSensorVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
      ahrs.getRotation2d(), left1.getSelectedSensorPosition(), right1.getSelectedSensorPosition(), pose);
  }

  public CommandBase resetEncoders() {
    return runOnce(() -> {
      left1.setSelectedSensorPosition(0);
      right1.setSelectedSensorPosition(0);
    });
  }

  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              PathPlanConstants.ksVolts,
              PathPlanConstants.kvVoltSecondsPerMeter,
              PathPlanConstants.kaVoltSecondsSquaredPerMeter),
              PathPlanConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
          PathPlanConstants.kMaxSpeedMetersPerSecond,
          PathPlanConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(PathPlanConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    double ahrsInitialPosition = getRobotYaw();
    Trajectory traj =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 1, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            traj,
            RobotContainer.m_Drivetrain::getPose,
            new RamseteController(PathPlanConstants.kRamseteB, PathPlanConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              PathPlanConstants.ksVolts,
              PathPlanConstants.kvVoltSecondsPerMeter,
              PathPlanConstants.kaVoltSecondsSquaredPerMeter),
              PathPlanConstants.kDriveKinematics,
              RobotContainer.m_Drivetrain::getWheelSpeeds,
            new PIDController(1/500, 0, 0),
            new PIDController(1/500, 0, 0),
            // RamseteCommand passes volts to the callback
            RobotContainer.m_Drivetrain::tankDriveVolts,
            RobotContainer.m_Drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    RobotContainer.m_Drivetrain.resetOdometry(traj.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> RobotContainer.m_Drivetrain.tankDriveVolts(0, 0));
  }

  public double getAverageEncoderDistance() {
    return (left1.getSelectedSensorPosition() + right1.getSelectedSensorPosition()) / 2.0;
  }

  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -ahrs.getRate();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left1.setVoltage(leftVolts);
    right1.setVoltage(rightVolts);
    m_drive.feed();
  }
  
  public CommandBase invertDrive(){ //swap front and back of robot
    return runOnce(()->{
      invertedDrive = !invertedDrive;
    });

  }

  public CommandBase halfSpeed(){
    return runOnce(() -> {
      halfSpeed = !halfSpeed;
    });
  }
  public double speedMultiplier = .5;
  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    if(!halfSpeed){
      speedMultiplier = 0.75;
    }else{
      speedMultiplier = 1;
    }
    if(!invertedDrive){ //check for invert
      drive.arcadeDrive(moveSpeed  * speedMultiplier, rotateSpeed * speedMultiplier); //if no move forward (battery side)

    }else{
      drive.arcadeDrive(moveSpeed * speedMultiplier, -1*rotateSpeed  * speedMultiplier); //if yes forward becomes back (non-battery becomes forward)
    }
  }
  
  PIDController leftController = new PIDController(1.0 / 300.0, 0, 0);
  PIDController rightController = new PIDController(1.0 / 300.0, 0, 0);


  public void pidDrive(double moveSpeed, double rotateSpeed) {

    leftController.setTolerance(150);
    rightController.setTolerance(150);

    if(!halfSpeed){
      speedMultiplier = 0.75;
    }else{
      speedMultiplier = 1;
    }
    if(!invertedDrive){ //check for invert
      pidArcade(moveSpeed  * speedMultiplier, rotateSpeed * speedMultiplier); //if no move forward (battery side)

    }else{
      pidArcade(-1 * moveSpeed * speedMultiplier, rotateSpeed  * speedMultiplier); //if yes forward becomes back (non-battery becomes forward)
    }
  }

  public void pidArcade(double move, double rotate) {
    //Setpoint
    leftController.setSetpoint(move - rotate);
    rightController.setSetpoint((move + rotate));

    SmartDashboard.putNumber("Left Setpoint: " , leftController.getSetpoint());
    SmartDashboard.putNumber("Right Setpoint: " , rightController.getSetpoint());
    
    SmartDashboard.putNumber("Left Velocity: ", -left1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity: " , right1.getSelectedSensorVelocity());
    
    //Velocity
    double leftAccel = leftController.calculate(-left1.getSelectedSensorVelocity());
    double rightAccel = rightController.calculate(right1.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Left Speed: " , leftAccel);
    SmartDashboard.putNumber("Right Speed: " , rightAccel);

    leftVelocity += leftAccel * 0.02;
    rightVelocity += rightAccel * 0.02;

    //Output
    left1.set(ControlMode.PercentOutput, -leftVelocity);
    left2.set(ControlMode.PercentOutput, -leftVelocity);
    right1.set(ControlMode.PercentOutput, rightVelocity);
    right2.set(ControlMode.PercentOutput, rightVelocity);

    //drive.tankDrive(leftController.calculate(left1.getSelectedSensorVelocity()), rightController.calculate(right1.getSelectedSensorVelocity()));
  }

  public void DriveTank(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void tankDrive(double left, double right){ //controls right and left speeds independantly
    drive.tankDrive(left, right);
  }

  public CommandBase toggleBrake(){
    return runOnce(() -> {
      brakeModeBool = !brakeModeBool;
      if (RobotContainer.m_Drivetrain.brakeModeBool) {
        RobotContainer.m_Drivetrain.setBrakeMode();
      } else {
        RobotContainer.m_Drivetrain.setCoastMode();
      }
    });
  }

  public double getLeftEncoder () {
    return left1.getSelectedSensorPosition();
  }

  public double getRightEncoder () {
    return right1.getSelectedSensorPosition();
  }

  public CommandBase resetLeftEncoder () {
    return runOnce(() -> {
      left1.setSelectedSensorPosition(0);
      left2.setSelectedSensorPosition(0);
    });
  }

  public CommandBase resetRightEncoder () {
    return runOnce(() -> {
      right1.setSelectedSensorPosition(0);
      right2.setSelectedSensorPosition(0);
    });
  }


  double leftSpeed = .40;
  double rightSpeed = .40;
  public boolean driveDistance(int inches){

    //2048*6/8 = 18"
    //2048*12/6 = 
    double EncoderCountDist = -1*inches*2048*6/8/1.8; //constant found from encodercount*wheel Diamater*gear ratio * inches
    // System.out.println("DRIVE DISTANCE");
    System.out.println("CURRENT: " + left1.getSelectedSensorPosition() + " Goal: " + EncoderCountDist);


      if(left1.getSelectedSensorPosition() > EncoderCountDist){ //if we haven't reached encoder count
        if(-1*left1.getSelectedSensorPosition() > right1.getSelectedSensorPosition()){ //if right is moving slower than left
          rightSpeed += .0015; //speed up right
        }else if(-1*left1.getSelectedSensorPosition() < right1.getSelectedSensorPosition()){ //if left is slower than right
          rightSpeed -= .0015; //slow down right
        }else{
          //do nothing
        }
        drive.tankDrive(-leftSpeed, rightSpeed); //drive with adjusted speeds
        return false;
      }else{
        drive.tankDrive(0, 0); //at the end stop drivetrain
        initializeEncoders(); //reset the encoders back to 0
        return true;
      }
  }
  boolean firstTimeback = true;
  public boolean driveDistanceBack(int inches){
    if(firstTimeback){
      leftSpeed = .48;
      rightSpeed = .48;
      firstTimeback = false;
    }
    //2048*6/8 = 18"
    //2048*12/6 = 
    double EncoderCountDist = inches*2048*6/8/1.8; 
    System.out.println("DRIVE DISTANCE");
    System.out.println("CURRENT: " + left1.getSelectedSensorPosition() + " Goal: " + EncoderCountDist);


      if(left1.getSelectedSensorPosition() < EncoderCountDist){
        if(-1*left1.getSelectedSensorPosition() < right1.getSelectedSensorPosition()){
          rightSpeed += .0022;
        }else if(-1*left1.getSelectedSensorPosition() > right1.getSelectedSensorPosition()){
          rightSpeed -= .0022;
        }else{
          //do nothing
        }
        drive.tankDrive(leftSpeed, -rightSpeed);
        return false;
      }else{
        drive.tankDrive(0, 0);
        return true;
      }
  }

  public void zeroGyro(){
    // ahrs.zeroYaw();
    // ahrs.set
  }
  public double getRobotPitch() {
    return ahrs.getPitch();
  }

  public double getRobotRoll() {
    return ahrs.getRoll();
  }

  public double getRobotYaw() {
    return ahrs.getAngle();
  }

  public String getRobotOdometry() {
    String string = "Degrees: " + m_odometry.getPoseMeters().getRotation().getDegrees()
    + " \nX: " + m_odometry.getPoseMeters().getX() + "\n Y: " + m_odometry.getPoseMeters().getY();
    return string;
  }

  public CommandBase resetGyro() {
    return runOnce(() -> {
      ahrs.reset();
    });  
  }

  public void initializeEncoders(){
    left1.setSelectedSensorPosition(0);
    left2.setSelectedSensorPosition(0);
    right1.setSelectedSensorPosition(0);
    right2.setSelectedSensorPosition(0);
  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void setBrakeMode(){ //sets all motors into brake mode (when stopped, robot slams stop)
    left1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
  }

  public CommandBase setBrakeModeAuto(){
    return runOnce(() -> {
      left1.setNeutralMode(NeutralMode.Brake);
      left2.setNeutralMode(NeutralMode.Brake);
      right1.setNeutralMode(NeutralMode.Brake);
      right2.setNeutralMode(NeutralMode.Brake);
    });
  }

  public void setCoastMode(){ //sets all to coast mode (when stopped, robot will roll to a stop)
    left1.setNeutralMode(NeutralMode.Coast);
    left2.setNeutralMode(NeutralMode.Coast);
    right1.setNeutralMode(NeutralMode.Coast);
    right2.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        ahrs.getRotation2d(), left1.getSelectedSensorPosition() - left1PreviousEncoder, right1.getSelectedSensorPosition() - right1PreviousEncoder);
    left1PreviousEncoder = left1.getSelectedSensorPosition();
    right1PreviousEncoder = right1.getSelectedSensorPosition();
    SmartDashboard.putNumber("Gyro", getRobotYaw());
    SmartDashboard.putData(ahrs);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
