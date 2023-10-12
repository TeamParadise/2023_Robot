// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj.SerialPort;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorPortConstants;
import frc.robot.RobotContainer;




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

  WPI_Pigeon2 imu;
  DifferentialDriveOdometry m_Odometry;
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23.5));

  public Drivetrain() {
    imu = new WPI_Pigeon2(15);

    
    m_Odometry = new DifferentialDriveOdometry(
    imu.getRotation2d(),
    getLeftEncoder(), getRightEncoder(),
    new Pose2d(0, 0, new Rotation2d()));
    // left1.setSafetyEnabled(true);
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
  
  PIDController leftController = new PIDController(1.0 / 204.8, 0, 1/150);
  PIDController rightController = new PIDController(1.0 / 204.8, 0, 1/150);

  
  PIDController leftControllerD = new PIDController(0, 0, 1/150);
  PIDController rightControllerD = new PIDController(0, 0, 1/150);



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
    return left2.getSelectedSensorPosition();
  }

  public double getLeftEncoderVelocity () {
    return (left1.getSelectedSensorVelocity() / 2048) * 2 * Math.PI * Units.inchesToMeters(6);
  }

  public double getRightEncoderVelocity () {
    return (right1.getSelectedSensorVelocity() / 2048) * 2 * Math.PI * Units.inchesToMeters(6);
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

  public CommandBase resetEncoders() {
    return runOnce(() -> {
      right1.setSelectedSensorPosition(0);
      right2.setSelectedSensorPosition(0);
      left1.setSelectedSensorPosition(0);
      left2.setSelectedSensorPosition(0);
    });
  }

  

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
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
    return imu.getPitch();
  }

  public double getRobotRoll() {
    return imu.getRoll();
  }

  public double getRobotAngle() {
    return imu.getRotation2d().getDegrees();
  }

  public Rotation2d getRobotAngle2d() {
    return imu.getRotation2d();
  }

  // public String getRobotOdometry() {
  //   String string = "Degrees: " + m_Odometry.getPoseMeters().getRotation().getDegrees()
  //   + " \nX: " + m_Odometry.getPoseMeters().getX() + "\n Y: " + m_Odometry.getPoseMeters().getY();
  //   return string;
  // }

  public CommandBase resetGyro() {
    return runOnce(() -> {
      imu.reset();
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

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  // public void driveChassisSpeed(double vx, double vy, double omega){
  //   DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vx, vy, omega));
    
  //   double leftVelocity = wheelSpeeds.leftMetersPerSecond;
  //   double rightVelocity = wheelSpeeds.rightMetersPerSecond;

  //   left1.set(ControlMode.Velocity, leftVelocity);
  //   left2.follow(left1);

  //   right1.set(ControlMode.Velocity, rightVelocity);
  //   right2.follow(right1);

  // }

  public void driveVelocity(double left, double right){
    
    // left1.set(ControlMode.Velocity, left);
    // left2.follow(left1);

    System.out.println(left);



    // right1.set(ControlMode.Velocity, right);
    // right2.follow(right1);

    System.out.println(right);

  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_Odometry.resetPosition(imu.getRotation2d(), getLeftEncoder(), getRightEncoder(), pose);
  }

  public void resetPose() {
    resetEncoders();
    m_Odometry.resetPosition(imu.getRotation2d(), getLeftEncoder(), getRightEncoder(), new Pose2d());
  }

  Supplier<Pose2d> getPose = () -> {
     return m_Odometry.getPoseMeters();
  };

  BiConsumer<Double, Double>  driveVolts = (left, right) -> {
    left1.setVoltage(-left);
    left2.setVoltage(-left);
    right1.setVoltage(right);
    right2.setVoltage(right);

    System.out.println(left);
    System.out.println(right);


  };


  

  Supplier<DifferentialDriveWheelSpeeds> getCurrentSpeeds = () -> {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity() , -getRightEncoderVelocity());
  };


  
  public Command followTrajectoryCommand(String traj) {
    PathPlannerTrajectory AutoPath = PathPlanner.loadPath(traj, new PathConstraints(2 , 2));
    return new PPRamseteCommand(
          AutoPath,
          getPose,
          new RamseteController(0.07, 0),
          new SimpleMotorFeedforward(0.02, 0),
          kinematics,
          getCurrentSpeeds,
          new PIDController(0.85, 0, 0),
          new PIDController(0.85, 0, 0),
          driveVolts,
          true
          );
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getLeftEncoderVelocity() + " " + -getRightEncoderVelocity());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
