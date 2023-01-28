// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.Direction;

public class Drivetrain extends SubsystemBase {
  // TODO Placeholder constants.
  private static final double TICKS_PER_REVOLUTION = 42;
  private static final double WHEEL_DIAMETER = 5.0;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 9.89 / 1;
  private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);

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

  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;
  public static double targetPosition;
  public static Direction targetDirection;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(Constants.MOTOR_ID_0, null);
    leftRear = new CANSparkMax(Constants.MOTOR_ID_1, null);
    leftRear.follow(leftFront);

    rightFront = new CANSparkMax(Constants.MOTOR_ID_2, null);
    rightRear = new CANSparkMax(Constants.MOTOR_ID_3, null);
    rightRear.follow(rightFront);
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

//    leftFront.set(ControlMode.Position, targetPosition);
//    rightFront.set(ControlMode.Position, targetPosition);
    tankDrive(targetPosition, targetPosition);
  }

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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
