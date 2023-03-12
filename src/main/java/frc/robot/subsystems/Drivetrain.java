// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Direction;
@SuppressWarnings("Serial Warnings")

public class Drivetrain extends SubsystemBase {
  private boolean slow = false;
  private boolean straight = false;
  private static final double MAX_VELOCITY = 350;
	private static final double SLOW_VELOCITY = 650;
  private static double peakOutput = 0.2;

  // TODO Placeholder constants.
  private static final double TICKS_PER_REVOLUTION = 43;
  private static final double WHEEL_DIAMETER = 5.0;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 8.8984;
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
  public static final double RIGHT_POSITION_P = 0.065; 
  public static final double LEFT_POSITION_P = 0.01;

  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;
  public static double targetPosition;
  public static Direction targetDirection;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkMaxPIDController rightFrontPID;
  private SparkMaxPIDController leftFrontPID;



  //navXAhrs = new AHRS(SPI.Port.kMXP);

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    // setting left and right motors with a motor controller 
    leftFront = new CANSparkMax(Constants.LEFTFRONT_MOTOR, MotorType.kBrushless);
    leftFront.setSmartCurrentLimit(40);
    leftEncoder = leftFront.getEncoder();
    leftRear = new CANSparkMax(Constants.LEFTREAR_MOTOR, MotorType.kBrushless);
    leftRear.setSmartCurrentLimit(40);
    leftFront.setInverted(true);
    leftRear.follow(leftFront);
    rightFront = new CANSparkMax(Constants.RIGHTFRONT_MOTOR, MotorType.kBrushless);
    rightFront.setSmartCurrentLimit(40);
    rightEncoder = rightFront.getEncoder();
    rightRear = new CANSparkMax(Constants.RIGHTREAR_MOTOR, MotorType.kBrushless);
    rightRear.setSmartCurrentLimit(40);
    rightRear.follow(rightFront);

    // conversion factors for the enconders 
    leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    //leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    //rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);

    // setting PID for 
    rightFrontPID = leftFront.getPIDController();
    leftFrontPID = rightFront.getPIDController();

    leftFrontPID.setOutputRange(-0.5, 0.5);
    rightFrontPID.setOutputRange(-0.5, 0.5);

    setPositionPID(RIGHT_POSITION_P, LEFT_POSITION_P, POSITION_I, POSITION_D, POSITION_FEED_FORWARD);
    setVelocityPID(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_FEED_FORWARD);
  }

  public void tankDrive(double left, double right) {
    leftFront.set(left);
    rightFront.set(right);
  }

  public void driveDistance(double inches, Direction direction) {
    targetDirection = direction;
    if (direction == Direction.FORWARD) {
      targetPosition = -inches;
    } else if (direction == Direction.BACKWARD) {
      targetPosition = inches;
    } else {
      targetPosition = 0;
    }

    SmartDashboard.putNumber("SetPoint", targetPosition);
    SmartDashboard.putNumber( "Left Position", leftEncoder.getPosition());
    SmartDashboard.putNumber( "Right Position", rightEncoder.getPosition());

    rightFrontPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    leftFrontPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  private void configController(CANSparkMax sparkMax, double kP, double kI, double kD, double kF){
    SparkMaxPIDController controller = sparkMax.getPIDController();
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kF);
    controller.setOutputRange(-peakOutput, peakOutput);
    sparkMax.setCANTimeout(100);
  }

  public void configAllControllers(double kP, double kI, double kD, double kF) {
    configController(rightFront, kP, kI, kD, kF);
    configController(rightRear, kP, kI, kD, kF);
    configController(leftFront, kP, kI, kD, kF);
    configController(leftRear, kP, kI, kD, kF);
  }

  // creates a PID velocity robot. Uses PID settings to determine speeds
//   public void tankDriveVelocity(double left, double right) {
//     double targetLeft;
//     double targetRight;

//     // max rpm of wheels desired
//     double targetVelocity = slow ? SLOW_VELOCITY : MAX_VELOCITY;

//     // target speed in encoder units based on joystick position
//     targetLeft = (left + 0.0078125) * targetVelocity * TICKS_PER_INCH;
//     targetRight = (right + 0.0078125) * targetVelocity * TICKS_PER_INCH;

//     // set target speeds to motors
//     leftFront.getPIDController().setReference(targetLeft, ControlType.kVelocity);
//     rightFront.getPIDController().setReference(targetRight, ControlType.kVelocity);

//     // SmartDashboard.putNumber("left:", getPosition());
//     // SmartDashboard.putNumber("right:", getPosition());
// }
  
public void tankDriveVelocity(double leftVelocity, double rightVelocity){
  rightFrontPID.setReference(rightVelocity, CANSparkMax.ControlType.kVelocity);
  leftFrontPID.setReference(leftVelocity, CANSparkMax.ControlType.kVelocity);
}

  public void tankPercent(double left, double right) {
    tankDriveVelocity(left * 0.75, right * 0.75);
  }  

  public void setPeakOutput(double output) {
    peakOutput = output;
  }

  public double getPosition() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
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

  public void resetVelocity() {
    rightFrontPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    leftFrontPID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void setPositionPID(double kPR, double kPL, double kI, double kD, double kF) {
    rightFrontPID.setP(kPR);
    rightFrontPID.setI(kI);
    rightFrontPID.setD(kD);
    rightFrontPID.setFF(kF);
    rightFront.setCANTimeout(100);

    leftFrontPID.setP(kPL);
    leftFrontPID.setI(kI);
    leftFrontPID.setD(kD);
    leftFrontPID.setFF(kF);
    leftFront.setCANTimeout(100);
  }

  public void setVelocityPID(double kP, double kI, double kD, double kF) {
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

  public void balanceOnStation() {
    if(RobotContainer.navX.getPitch() < -5) {
      tankDrive(0.4, 0.4);
    }
    else if(RobotContainer.navX.getPitch() > 5) {
      tankDrive(-0.4, -0.4);
    }
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
 public void resetAngle(){
     RobotContainer.navX.reset();
   }

   public double getAngle(){
     return RobotContainer.navX.getAngle();
   }

   public void angleTurn(Direction direction){
     double speed = 0.2;
     
     if (direction == Direction.RIGHT) {
      leftFront.set(-speed);
      rightFront.set(speed);
    } else if (direction == Direction.LEFT) {
      leftFront.set(speed);
      rightFront.set( -speed);
    } else {
      leftFront.set(0);
      rightFront.set(0);
    }
  }
}
