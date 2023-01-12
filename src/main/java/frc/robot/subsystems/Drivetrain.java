// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax; 
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
