// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevatorMotor;

  public Elevator() {
      elevatorMotor = new CANSparkMax(Constants.MOTOR_ID_4, MotorType.kBrushless);
      // elevatorMotor.set(preset);
    }

    public void setHeight(int preset) {
      double current = elevatorMotor.getEncoder().getPosition();
      double change = height[preset] - current;
      current = height[preset];
      elevatorMotor.getPIDController().setReference(change, ControlType.kPosition);
      elevatorMotor.getEncoder().setPosition(0);
      // TODO 2/18
    }
    //private void configController(CANSparkMax sparkMax, double kP, double kI, double kD, double kF)
    {
      
      //SparkMaxPIDController controller = sparkMax.getPIDController();
      //controller.setP(kP);
      //controller.setI(kI);
      //controller.setD(kD);
      //controller.setFF(kF);
      //controller.setOutputRange(-peakOutput, peakOutput);
      //sparkMax.setCANTimeout(100);
      //TO DO pid later
    
   
}

