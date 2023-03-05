// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.Direction;
@SuppressWarnings("Serial Warnings")
public class Drivetrain extends SubsystemBase {

  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(Constants.LEFTFRONT_MOTOR, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.LEFTREAR_MOTOR, MotorType.kBrushless);
    leftFront.setInverted(true);
    leftRear.follow(leftFront);

    rightFront = new CANSparkMax(Constants.RIGHTFRONT_MOTOR, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.RIGHTREAR_MOTOR, MotorType.kBrushless);
    rightFront.setInverted(true);
    rightRear.follow(rightFront);
  }

  public void tankDrive(double left, double right) {

    leftFront.set(left*1/2);
    rightFront.set(-right*1/2);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}