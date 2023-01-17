// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants;
import frc.robot.Direction;
// import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
  private boolean slow = false;
  private boolean straight = false;
  private static final double MAX_VELOCITY = 350;
	private static final double SLOW_VELOCITY = 650;
  private static double peakOutput = 0.2;

  // TODO Placeholder constants.
  private static final double TICKS_PER_REVOLUTION = 4096;
  private static final double WHEEL_DIAMETER = 6.0;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 10.7 / 1;
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
  //public AHRS navX;

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
    leftFront.getPIDController().setReference(left, ControlType.kPosition);
    rightFront.getPIDController().setReference(-right, ControlType.kPosition);
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

    leftFront.getPIDController().setReference(targetPosition, ControlType.kPosition);
    rightFront.getPIDController().setReference(targetPosition, ControlType.kPosition);
  }

  public void setPID(double kP, double kI, double kD, double kF) {
    setControllerPID(rightFront, kP, kI, kD, kF);
    setControllerPID(rightRear, kP, kI, kD, kF);
    setControllerPID(leftFront, kP, kI, kD, kF);
    setControllerPID(leftRear, kP, kI, kD, kF);
  }

  private void setControllerPID(CANSparkMax sparkMax, double kP, double kI, double kD, double kF)
  {
    SparkMaxPIDController controller = sparkMax.getPIDController();
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kF);
    controller.setOutputRange(-1, 1); // TODO move to constant
    sparkMax.setCANTimeout(100);
  }

  // creates a PID velocity robot. Uses PID settings to determine speeds
  public void tankDriveVelocity(double left, double right) {
    double targetLeft;
    double targetRight;

    // max rpm of wheels desired
    double targetVelocity = slow ? SLOW_VELOCITY : MAX_VELOCITY;

    // target speed in encoder units based on joystick position
    targetLeft = (left + 0.0078125) * targetVelocity * TICKS_PER_INCH;
    targetRight = (right + 0.0078125) * targetVelocity * TICKS_PER_INCH;

    // set target speeds to motors
    leftFront.getPIDController().setReference(targetLeft, ControlType.kVelocity);
    rightFront.getPIDController().setReference(targetRight, ControlType.kVelocity);

    //SmartDashboard.putNumber("left:", getPosition());
    //SmartDashboard.putNumber("right:", getPosition());
}
  
  public void tankPercent(double left, double right) {
    tankDriveVelocity(left * 0.75, right * 0.75);
  }  

  public void setPeakOutput(double output) {
    peakOutput = output;
  }

  public double getPosition() {
    return rightFront.getEncoder().getPosition();
  }

  public double getVelocity() {
    return rightFront.getEncoder().getVelocity() / GEAR_RATIO / 2048;
  }

  public void resetPosition() {
    leftFront.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);
    leftRear.getEncoder().setPosition(0);
    rightRear.getEncoder().setPosition(0);
  }

  public double getAngle() {
		//return navX.getAngle();
    return 0; // TODO
	}

	public void resetAngle() {
		//navX.reset();
	}

  public void angleTurn(Direction direction) {
    double speed = 0.2;

    //SmartDashboard.putNumber("angle: ", getAngle());
    if (direction == Direction.RIGHT) {
      leftFront.set(-speed);
      rightFront.set(speed);
    } else if (direction == Direction.LEFT) {
      leftFront.set(speed);
      rightFront.set(-speed);
    } else {
      leftFront.set(0);
      rightFront.set(0);
    }
  }

  public void setSlow(boolean val) {
    slow = val;
  }

  public void setStraight(boolean val) {
    straight = val;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
