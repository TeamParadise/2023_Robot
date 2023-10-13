// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canDeviceIds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.util.Units;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WPI_TalonFX right1 = new WPI_TalonFX(canDeviceIds.DRIVE_TRAIN_RIGHT_1);
  public WPI_TalonFX right2 = new WPI_TalonFX(canDeviceIds.DRIVE_TRAIN_RIGHT_2);

  public WPI_TalonFX left1 = new WPI_TalonFX(canDeviceIds.DRIVE_TRAIN_LEFT_1);
  public WPI_TalonFX left2 = new WPI_TalonFX(canDeviceIds.DRIVE_TRAIN_LEFT_2);

  MotorControllerGroup leftMotors = new MotorControllerGroup(left1, left2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(right1, right2);

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  DifferentialDriveKinematics kinematics;
  DifferentialDriveOdometry m_Odometry;

  WPI_Pigeon2 imu;

  Field2d m_field;

  public double speedMultiplier = 1;
  public boolean brakeModeBool = true;

  public Drivetrain() {
    resetEncoders();
    imu = new WPI_Pigeon2(canDeviceIds.PIDGEON_2_PORT);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23.5));
    m_Odometry = new DifferentialDriveOdometry(imu.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), new Pose2d(0, 0, new Rotation2d()));
    m_field = new Field2d();
    Pigeon2Configuration config = new Pigeon2Configuration();
    imu.configAllSettings(config);


    SmartDashboard.putData("Field", m_field);
  }

  public void arcadeDrive(double rotateSpeed, double moveSpeed) {
    drive.arcadeDrive(rotateSpeed * Math.abs(speedMultiplier), moveSpeed  * speedMultiplier); //if yes forward becomes back (non-battery becomes forward)
  }

  public void tankDrive(double left, double right){ //controls right and left speeds independantly
    drive.tankDrive(left, right);
  }

  BiConsumer<Double, Double>  driveVolts = (left, right) -> {
    left1.setVoltage(left);
    left2.setVoltage(left);
    right1.setVoltage(-right);
    right2.setVoltage(-right);
    SmartDashboard.putNumber("leftVoltage", left);
    SmartDashboard.putNumber("rightVoltage",right);
  };

  public CommandBase invertDrive(){ //swap front and back of robot
    return runOnce(()->{
      speedMultiplier = speedMultiplier *-1;
    }); 
  }

  public CommandBase halfSpeed(){
    return runOnce(() -> {
      if (Math.abs(speedMultiplier) < 1) speedMultiplier = Math.abs(speedMultiplier)/speedMultiplier;
      else speedMultiplier *= 0.75;
    });
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

  Supplier<Pose2d> getPose = () -> {
    return m_Odometry.getPoseMeters();
  };

  public Pose2d getFieldPos(){
    return m_Odometry.getPoseMeters();
    }

  Supplier<DifferentialDriveWheelSpeeds> getCurrentSpeeds = () -> {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity() , getRightEncoderVelocity());
  };
  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_Odometry.resetPosition(imu.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }

  public void resetPose() {
    m_Odometry.resetPosition(imu.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), new Pose2d());
  }
  
  public void zeroGyro(){
    imu.reset();
    System.out.println(imu.getRotation2d().unaryMinus());
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

  public CommandBase resetGyro() {
    return runOnce(() -> {
      imu.reset();
    });  
  }

  public double getLeftEncoder () {
    return left1.getSelectedSensorPosition();
  }

  public double getLeftEncoderVelocity () {
    // return (left1.getSelectedSensorVelocity() / 2048) * 2 * Math.PI * Units.inchesToMeters(6);
    return -(2 * Math.PI * Units.inchesToMeters(3) * ((left1.getSelectedSensorVelocity() / 2048) / 8.45) * 10);
  }

  public double getLeftEncoderMeters () {
    // return (left1.getSelectedSensorVelocity() / 2048) * 2 * Math.PI * Units.inchesToMeters(6);
    return (2 * Math.PI * Units.inchesToMeters(3) * ((left1.getSelectedSensorPosition() / 2048) / 8.45));
  }

  public double getRightEncoder () {
    return left2.getSelectedSensorPosition();
  }

  public double getRightEncoderVelocity () {
    // return (right1.getSelectedSensorVelocity() / 2048) * 2 * Math.PI * Units.inchesToMeters(6);
    return (2 * Math.PI * Units.inchesToMeters(3) * ((right1.getSelectedSensorVelocity() / 2048) / 8.45) * 10);
  }

  public double getRightEncoderMeters () {
    // return (right1.getSelectedSensorVelocity() / 2048) * 2 * Math.PI * Units.inchesToMeters(6);
    return (2 * Math.PI * Units.inchesToMeters(3) * ((left1.getSelectedSensorPosition() / 2048) / 8.45));
  }

  public CommandBase resetEncoders() {
    return runOnce(() -> {
      right1.setSelectedSensorPosition(0);
      right2.setSelectedSensorPosition(0);
      left1.setSelectedSensorPosition(0);
      left2.setSelectedSensorPosition(0);
    });
  }

  public void initializeEncoders(){
    left1.setSelectedSensorPosition(0);
    left2.setSelectedSensorPosition(0);
    right1.setSelectedSensorPosition(0);
    right2.setSelectedSensorPosition(0);
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

  public Command followTrajectoryCommand(String traj) {
    PathPlannerTrajectory AutoPath = PathPlanner.loadPath(traj, new PathConstraints(3 , 3));

    this.resetOdometry(AutoPath.getInitialPose());

    return new PPRamseteCommand(
          AutoPath,
          getPose,
          new RamseteController(0.7, 0.0),
          new SimpleMotorFeedforward(0, 0),
          kinematics,
          getCurrentSpeeds,
          new PIDController(0.75, 0, 0),
          new PIDController(0.75, 0, 0),
          driveVolts,
          true,
          this
          );
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
      }
      else if(-1*left1.getSelectedSensorPosition() < right1.getSelectedSensorPosition()){ //if left is slower than right
        rightSpeed -= .0015; //slow down right
      }
      drive.tankDrive(-leftSpeed, rightSpeed); //drive with adjusted speeds
      return false;
      }
      else{
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
        }
        else if(-1*left1.getSelectedSensorPosition() > right1.getSelectedSensorPosition()){
          rightSpeed -= .0022;
        }
        drive.tankDrive(leftSpeed, -rightSpeed);
        return false;
      }
      else{
        drive.tankDrive(0, 0);
        return true;
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Left Position Meters", getLeftEncoderMeters());
    SmartDashboard.putNumber("Right Position Meters", getRightEncoderMeters());
    m_Odometry.update(getRobotAngle2d(), getLeftEncoderMeters(), getRightEncoderMeters());
    m_field.setRobotPose(getFieldPos());
  }

  // @Override
  // public void simulationPeriodic() {};
}
