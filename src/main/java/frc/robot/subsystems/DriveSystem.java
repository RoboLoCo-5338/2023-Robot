// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.commands.Direction;

public class DriveSystem extends PIDSubsystem {
	private static final double MAX_VELOCITY = 350;
	private static final double SLOW_VELOCITY = 650;
	private static double PEAK_OUTPUT = 0.2;
  public static boolean slow = false;
  public static boolean straight = false;

  // set PID values for teleop
  public static final double VELOCITY_P = 0.0110;
	public static final double VELOCITY_I = 0.0;
	public static final double VELOCITY_D = 0.0;
	public static final double VELOCITY_FEED_FORWARD = 0.0;

  // set PID values for autonomous
	public static final double POSITION_P = 0.0175821;
	public static final double POSITION_I = 0.0;
	public static final double POSITION_D = 0.0020951;
	public static final double POSITION_FEED_FORWARD = 0.0;

  // encoder math
  private static final double TICKS_PER_REVOLUTION = 2048;
  private static final double WHEEL_DIAMETER = 6.0;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 10.7 / 1;
  private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);

  public static double targetPosition = 0;
	private static Direction targetDirection;
  private static final int DEFAULT_TIMEOUT = 30;
  

  /** Creates a new DriveSystem. */
  private static WPI_TalonFX rightFront;
  private static WPI_TalonFX rightRear;
  private static WPI_TalonFX leftFront;
  private static WPI_TalonFX leftRear;

  // create Gyro object


  public DriveSystem() {
    // set PID values here
    super(new PIDController(VELOCITY_P, VELOCITY_I, VELOCITY_D));

    rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT_DRIVE);
    rightRear = new WPI_TalonFX(Constants.RIGHT_REAR_DRIVE);
    leftFront = new WPI_TalonFX(Constants.LEFT_FRONT_DRIVE);
    leftRear = new WPI_TalonFX(Constants.LEFT_REAR_DRIVE);



    configureTalon();
  }

  
  // configure talon properties
  public static void configureTalon() {
    // JDE: Are current limits set - should they be set here or elsewhere?
    // https://docs.ctre-phoenix.com/en/latest/ch13_MC.html#new-api-in-2020
    rightFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
		rightFront.configNeutralDeadband(0.001, 0);
    rightFront.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    rightFront.configPeakOutputForward(PEAK_OUTPUT);
		rightFront.configPeakOutputReverse(-PEAK_OUTPUT);
    rightFront.configNominalOutputForward(0, 30);
		rightFront.configNominalOutputReverse(0, 30);

    leftFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
		leftFront.configNeutralDeadband(0.001, 0);
    leftFront.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    leftFront.setInverted(true);
    leftFront.setSensorPhase(true);
    leftFront.configPeakOutputForward(PEAK_OUTPUT);
		leftFront.configPeakOutputReverse(-PEAK_OUTPUT);
    leftFront.configNominalOutputForward(0, DEFAULT_TIMEOUT);
		leftFront.configNominalOutputReverse(0, DEFAULT_TIMEOUT);

    //rightRear.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    rightRear.follow(rightFront);
		rightRear.configNeutralDeadband(0.001, 0);
    rightRear.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    rightRear.follow(rightFront, FollowerType.AuxOutput1);
    rightRear.configPeakOutputForward(PEAK_OUTPUT);
		rightRear.configPeakOutputReverse(-PEAK_OUTPUT);
    rightRear.configNominalOutputForward(0, 30);
		rightRear.configNominalOutputReverse(0, 30);
    rightRear.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    rightRear.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    // leftRear.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    leftRear.follow(leftFront);
		leftRear.configNeutralDeadband(0.001, 0);
    leftRear.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    leftRear.setInverted(true);
    leftRear.setSensorPhase(true);
    leftRear.follow(leftFront, FollowerType.AuxOutput1);    
		leftRear.configPeakOutputForward(PEAK_OUTPUT);
		leftRear.configPeakOutputReverse(-PEAK_OUTPUT);
		leftRear.configNominalOutputForward(0, DEFAULT_TIMEOUT);
		leftRear.configNominalOutputReverse(0, DEFAULT_TIMEOUT);
    leftRear.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    leftRear.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  public void setPIDF(double kP, double kI, double kD, double kF) {
    rightFront.config_kP(0, kP, 100);
    rightFront.config_kI(0, kI, 100);
    rightFront.config_kD(0, kD, 100);
    rightFront.config_kF(0, kF, 100);

    leftFront.config_kP(0, kP, 100);
    leftFront.config_kI(0, kI, 100);
    leftFront.config_kD(0, kD, 100);
    leftFront.config_kF(0, kF, 100);
    
    rightRear.config_kP(0, kP, 100);
    rightRear.config_kI(0, kI, 100);
    rightRear.config_kD(0, kD, 100);
    rightRear.config_kF(0, kF, 100);

    leftRear.config_kP(0, kP, 100);
    leftRear.config_kI(0, kI, 100);
    leftRear.config_kD(0, kD, 100);
    leftRear.config_kF(0, kF, 100);
  }

   // creates a PID velocity robot. Uses PID settings to determine speeds
   public void tankDriveVelocity(double left, double right) {
    double targetLeft;
    double targetRight;

    // max rpm of wheels desired
    double targetVelocity = MAX_VELOCITY;
    
    if (slow) {
      targetVelocity = SLOW_VELOCITY;
    }

    // target speed in encoder units based on joystick position
    targetLeft = (left + 0.0078125) * targetVelocity * TICKS_PER_INCH;
    targetRight = (right + 0.0078125) * targetVelocity * TICKS_PER_INCH;

    // set target speeds to motors
    leftFront.set(ControlMode.Velocity, targetLeft);
    rightFront.set(ControlMode.Velocity, targetRight);

    SmartDashboard.putNumber("left:", getPosition());
    SmartDashboard.putNumber("right:", getPosition());
  }

  public void tankPercent(double left, double right) {
    leftFront.set(ControlMode.PercentOutput, left * 0.75);
    rightFront.set(ControlMode.PercentOutput, right * 0.75);
  }

  public void driveDistance(double inches, Direction direction) {
    targetDirection = direction;
		if (direction == Direction.FORWARD) {
			targetPosition = -1 * inches * TICKS_PER_INCH * GEAR_RATIO;
		} else if (direction == Direction.BACKWARD) {
			targetPosition = inches * TICKS_PER_INCH * GEAR_RATIO;
		} else {
			targetPosition = 0;
		}

		leftFront.set(ControlMode.Position, targetPosition);
		rightFront.set(ControlMode.Position, targetPosition);
  }

  public void setPeakOutput(double output) {
    PEAK_OUTPUT = output;
  }

  public double getPosition() {
    return rightFront.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return rightFront.getSelectedSensorVelocity() / GEAR_RATIO / 2048;
  }

  public void resetPosition() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    leftRear.setSelectedSensorPosition(0);
    rightRear.setSelectedSensorPosition(0);
  }


  public void angleTurn(Direction direction) {
    double speed = 0.2;


    if (direction == Direction.RIGHT) {
      leftFront.set(ControlMode.PercentOutput, -speed);
      rightFront.set(ControlMode.PercentOutput, speed);
    } else if (direction == Direction.LEFT) {
      leftFront.set(ControlMode.PercentOutput, speed);
      rightFront.set(ControlMode.PercentOutput, -speed);
    } else {
      leftFront.set(ControlMode.PercentOutput, 0);
      rightFront.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setSlow(boolean val) {
    slow = val;
  }

  public void setStraight(boolean val) {
    straight = val;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
