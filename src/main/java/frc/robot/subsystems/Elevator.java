// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;


import frc.robot.Constants;

public class Elevator extends SubsystemBase {


  private CANSparkMax sparkMax;
  public static double targetPosition;
  public static double changePosition;

  public Elevator() {
      sparkMax = new CANSparkMax(Constants.MOTOR_ID_3, MotorType.kBrushless);
    }

  public void change(double inches, Height height) {
      changePosition = targetPosition - height;
      sparkMax.set(changePosition);
  }



// subsystem should take button commands (1 button per preset) and give instructions to (spark?) motors
// 1. preset heights in constants, 2. methods in this class, 3. button -> method in commands
// class should be basically the same as drivetrain


/** Add your docs here. */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

