// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Joystick;



// import frc.robot.RobotContainer;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {


  private static final double Height = frc.robot.Constants.Height;
  private CANSparkMax sparkMax;
  public static double targetPosition;
  public static double changePosition;

  public Elevator(int port) {
      sparkMax = new CANSparkMax(Constants.MOTOR_ID_3, MotorType.kBrushless);
      Joystick name = new Joystick(port);

      
    }


  public void change() {
      changePosition = targetPosition - Height;
      sparkMax.set(changePosition);
  }


public boolean xhenPressed(){
  final RobotContainer Botcontainer = new RobotContainer();
  return Botcontainer.controller3.getTopPressed();
}




// subsystem should take button commands (1 button per preset) and give instructions to (spark?) motors
// 1. preset heights in constants, 2. methods in this class, 3. button -> method in commands
// class should be basically the same as drivetrain


/** Add your docs here. */
//public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.




}

