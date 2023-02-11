// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.commands.Direction;

import com.revrobotics.RelativeEncoder; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Drivetrain extends SubsystemBase {
  // TODO Placeholder constants.
  private static final double TICKS_PER_REVOLUTION = 4096;
  private static final double WHEEL_DIAMETER = 6.0;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 10.7 / 1;
  private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);

  // PID values for autonomous.
  public static final double POSITION_P = 0.0175821;
  public static final double POSITION_I = 0.0;
  public static final double POSITION_D = 0.0020951;
  public static final double POSITION_FEED_FORWARD = 0.0;

  // motors and encoders 
  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  public static double targetPosition;
  public static Direction targetDirection;
  private SparkMaxPIDController rightFrontPID; 
  private SparkMaxPIDController leftFrontPID;
  


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(Constants.MOTOR_ID_0, MotorType.kBrushless);
    leftEncoder = leftFront.getEncoder(); 
    leftRear = new CANSparkMax(Constants.MOTOR_ID_1, null);
    leftRear.follow(leftFront);

    rightFront = new CANSparkMax(Constants.MOTOR_ID_2, MotorType.kBrushless); // might need to change MotorType
    rightEncoder = rightFront.getEncoder(); 
    rightRear = new CANSparkMax(Constants.MOTOR_ID_3, null);
    rightRear.follow(rightFront);

    // set conversion factors to GEAR_RATIO
    leftEncoder.setPositionConversionFactor(GEAR_RATIO / WHEEL_CIRCUMFERENCE);
    rightEncoder.setVelocityConversionFactor(GEAR_RATIO / WHEEL_CIRCUMFERENCE);
    leftEncoder.setPositionConversionFactor(GEAR_RATIO / WHEEL_CIRCUMFERENCE);
    rightEncoder.setVelocityConversionFactor(GEAR_RATIO / WHEEL_CIRCUMFERENCE);

    rightFrontPID = rightFront.getPIDController();
    leftFrontPID = leftFront.getPIDController();

    // method to set PID values 
    this.setPID(POSITION_P, POSITION_I, POSITION_D, POSITION_FEED_FORWARD);
  }

  public void tankDrive(double left, double right) {
    leftFront.set(left);
    rightFront.set(-right);
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

    leftFrontPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    rightFrontPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setPID(double kP, double kI, double kD, double kF) {
    rightFrontPID.setP(kP);
    rightFrontPID.setI(kI);
    rightFrontPID.setD(kD);
    rightFrontPID.setFF(kF);
    rightFront.setCANTimeout(100);

    leftFrontPID.setP(kP);
    leftFrontPID.setI(kI);
    leftFrontPID.setD(kD);
    leftFrontPID.setFF(kF);
    leftFront.setCANTimeout(100);

  }

  public double getPosition() {
    double front = leftEncoder.getPosition() + rightEncoder.getPosition();
    // double rear = leftRear.getEncoder() + rightRear.getEncoder()
    // double avg = (front + rear) / 4.0;
    return front;
  }

  public void resetPosition(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
