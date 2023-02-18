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

public class Drivetrain extends SubsystemBase {

  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new CANSparkMax(Constants.MOTOR_ID_3, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.MOTOR_ID_2, MotorType.kBrushless);
    leftRear.follow(leftFront);

    rightFront = new CANSparkMax(Constants.MOTOR_ID_1, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.MOTOR_ID_0, MotorType.kBrushless);
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