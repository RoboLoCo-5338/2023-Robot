// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< Updated upstream
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.Direction;

public class Drivetrain extends SubsystemBase {
<<<<<<< Updated upstream
=======
  private boolean slow = false;
  private boolean straight = false;
  private static final double MAX_VELOCITY = 350;
	private static final double SLOW_VELOCITY = 650;
  private static double peakOutput = 0.2;
  private static final int DEFAULT_TIMEOUT = 30;
  

>>>>>>> Stashed changes
  // TODO Placeholder constants.
  private static final double TICKS_PER_REVOLUTION = 42;
  private static final double WHEEL_DIAMETER = 5.0;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 8.8984;
  private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);

  /* 
  // PID values for teleop.
  public static final double VELOCITY_P = 0.0110;
  public static final double VELOCITY_I = 0.0;
  public static final double VELOCITY_D = 0.0;
  public static final double VELOCITY_FEED_FORWARD = 0.0;

  // PID values for autonomous.
  public static final double POSITION_P = 0.0175821;
  public static final double POSITION_I = 0.0;
  public static final double POSITION_D = 0.0020951;
  public static final double POSITION_FEED_FORWARD = 0.0;
  */

  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;
  public static double targetPosition;
  public static Direction targetDirection;

<<<<<<< Updated upstream
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(Constants.MOTOR_ID_3, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.MOTOR_ID_2, MotorType.kBrushless);
=======
  public RelativeEncoder leftEncoder;
  public RelativeEncoder rightEncoder;
  public SparkMaxPIDController rightFrontPID;
  public SparkMaxPIDController leftFrontPID;


  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors ;

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors; 

  // The robot's drive
  private final DifferentialDrive m_drive;

  
  // The gyro sensor
  private AHRS Navx;
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  //navXAhrs = new AHRS(SPI.Port.kMXP);

  /** Creates a new Drivetrain. */
  public Drivetrain(){

    // setting left and right motors with a motor controller 
    leftFront = new CANSparkMax(Constants.LEFTFRONT_MOTOR, MotorType.kBrushless);
    leftFront.setSmartCurrentLimit(40);
    leftEncoder = leftFront.getEncoder();
    leftRear = new CANSparkMax(Constants.LEFTREAR_MOTOR, MotorType.kBrushless);
    leftRear.setSmartCurrentLimit(40);
    leftFront.setInverted(true);
>>>>>>> Stashed changes
    leftRear.follow(leftFront);

    rightFront = new CANSparkMax(Constants.MOTOR_ID_1, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.MOTOR_ID_0, MotorType.kBrushless);
    rightRear.follow(rightFront);
<<<<<<< Updated upstream
  }
=======

    // conversion factors for the enconders 
    leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    leftEncoder.setVelocityConversionFactor((WHEEL_CIRCUMFERENCE/GEAR_RATIO)/60);
    rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    rightEncoder.setVelocityConversionFactor((WHEEL_CIRCUMFERENCE/GEAR_RATIO)/60);

    // setting PID for 
    rightFrontPID = leftFront.getPIDController();
    leftFrontPID = rightFront.getPIDController();

    leftFrontPID.setOutputRange(-0.5, 0.5);
    rightFrontPID.setOutputRange(-0.5, 0.5);

    setPositionPID(RIGHT_POSITION_P, LEFT_POSITION_P, POSITION_I, POSITION_D, POSITION_FEED_FORWARD);
    setVelocityPID(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_FEED_FORWARD);

    Navx= new AHRS(SPI.Port.kMXP);

    m_leftMotors= new MotorControllerGroup(leftRear,leftFront);

    m_rightMotors=
    new MotorControllerGroup(
        rightRear,
        rightFront);

        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors); 

    resetEncoders();

    m_odometry =
    new DifferentialDriveOdometry(
        new Rotation2d(Navx.getAngle()),leftEncoder.getPosition()*0.0254*Math.PI, rightEncoder.getPosition()*0.0254*Math.PI);

          }
>>>>>>> Stashed changes

  public void tankDrive(double left, double right) {
    /*
    if (Math.abs(left) > 0.1){
      left = Math.signum(left)*0.1;
    }

    if (Math.abs(right) > 0.1){
      right = Math.signum(right)*0.1;
    }

    */

    leftFront.set(left*1/2);
    rightFront.set(-right*1/2);
    
  }

  public void driveDistance(double inches, Direction direction) {
    targetDirection = direction;
    if (direction == Direction.FORWARD) {
      targetPosition = -inches * TICKS_PER_INCH * GEAR_RATIO;
    } else if (direction == Direction.BACKWARD) {
      targetPosition = inches * TICKS_PER_INCH * GEAR_RATIO;
    } else {
      targetPosition = 0;
    }

//    leftFront.set(ControlMode.Position, targetPosition);
//    rightFront.set(ControlMode.Position, targetPosition);
    tankDrive(targetPosition, targetPosition);
  }
/* 
  public void setPID(double kP, double kI, double kD, double kF) {
    SparkMaxPIDController rightFrontPID = rightFront.getPIDController();
    rightFrontPID.setP(kP);
    rightFrontPID.setI(kI);
    rightFrontPID.setD(kD);
    rightFrontPID.setFF(kF);
    rightFront.setCANTimeout(100);

    SparkMaxPIDController leftFrontPID = leftFront.getPIDController();
    leftFrontPID.setP(kP);
    leftFrontPID.setI(kI);
    leftFrontPID.setD(kD);
    leftFrontPID.setFF(kF);
    leftFront.setCANTimeout(100);

    SparkMaxPIDController rightRearPID = rightRear.getPIDController();
    rightRearPID.setP(kP);
    rightRearPID.setI(kI);
    rightRearPID.setD(kD);
    rightFrontPID.setFF(kF);
    rightFront.setCANTimeout(100);

    SparkMaxPIDController leftRearPID = leftRear.getPIDController();
    leftRearPID.setP(kP);
    leftRearPID.setI(kI);
    leftRearPID.setD(kD);
    leftRearPID.setFF(kF);
    leftRear.setCANTimeout(100);
  }
*/


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
<<<<<<< Updated upstream
}
=======

  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      new Rotation2d(Navx.getAngle()), leftEncoder.getPosition()*0.1524, rightEncoder.getPosition()*0.1524);
      
      
    }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity()*0.1524, rightEncoder.getVelocity()*0.1524);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
  //   m_odometry.resetPosition(
  //     new Rotation2d(Navx.getAngle()),leftFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI,rightFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI, pose);
  // }

  m_odometry.resetPosition(
    new Rotation2d(Navx.getAngle()),0,0, pose);
}

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
   
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (rightEncoder.getPosition()*0.1524 + leftEncoder.getPosition()*0.1524) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    Navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return new Rotation2d(Navx.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -Navx.getRate();
  }

 

}
>>>>>>> Stashed changes
