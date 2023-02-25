// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


import frc.robot.Constants;
import frc.robot.commands.Direction;
public class Drivetrain extends SubsystemBase {
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
  //public static final double POSITION_P = 1;
  public static final double POSITION_I = 0;
  public static final double POSITION_D = 0.01;

  public static final double RIGHT_POSITION_P = 0.065; 
  public static final double LEFT_POSITION_P = 0.01;

  //public static final double RIGHT_POSITION_I = 0.0;
  //public static final double LEFT_POSITION_I = 0;

  //public static final double RIGHT_POSITION_D = 0;
  //public static final double LEFT_POSITION_D = 0;


  public static final double POSITION_FEED_FORWARD = 0;


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

  public AHRS navX;


  //navXAhrs = new AHRS(SPI.Port.kMXP);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(Constants.MOTOR_ID_3, MotorType.kBrushless);
    leftEncoder = leftFront.getEncoder();
    leftRear = new CANSparkMax(Constants.MOTOR_ID_2, MotorType.kBrushless);
    leftRear.follow(leftFront);

    leftFront.setInverted(true);


    rightFront = new CANSparkMax(Constants.MOTOR_ID_1, MotorType.kBrushless);
    rightEncoder = rightFront.getEncoder();
    rightRear = new CANSparkMax(Constants.MOTOR_ID_0, MotorType.kBrushless);
    rightRear.follow(rightFront);

    leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);
    rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEAR_RATIO);

    rightFrontPID = leftFront.getPIDController();
    leftFrontPID = rightFront.getPIDController();

    leftFrontPID.setOutputRange(-0.1, 0.1);
    rightFrontPID.setOutputRange(-0.1, 0.1);


    setPositionPID(RIGHT_POSITION_P, LEFT_POSITION_P, POSITION_I, POSITION_D, POSITION_FEED_FORWARD);
    setVelocityPID(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_FEED_FORWARD);

    navX = new AHRS(SPI.Port.kMXP);


  }

  public void tankDrive(double left, double right) {
    /*
    if (Math.abs(left) > 0.1){
      left = Math.signum(left)*0.1;
    }

    if (Math.abs(right) > 0.1){
      right = Math.signum(right)*0.1;
    }

    */

    leftFront.set(left*1/9);
    rightFront.set(-right*1/9);
    
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
    //rightFrontPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    //leftFrontPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity){
    rightFrontPID.setReference(rightVelocity, CANSparkMax.ControlType.kVelocity);
    leftFrontPID.setReference(leftVelocity, CANSparkMax.ControlType.kVelocity);
  }


    public double getPosition() {
      return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
    }
    
    public void resetPosition(){
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      SmartDashboard.putString("Hello", "hi");
    }
  
  public void resetVelocity(){
    rightFrontPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    leftFrontPID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

//    leftFront.set(ControlMode.Position, targetPosition);  
//    rightFront.set(ControlMode.Position, targetPosition);
  
 
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

   public void resetAngle(){
     navX.reset();
   }

   public double getAngle(){
     return navX.getAngle();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}