// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevatorMotor;
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController ;
  private double[] armHeights = new double[3];

  public static double armP=0.1;
  public static double armI=0.0;
  public static double armD=0.0;
  public static double armFeed_Forward=0.0;

  public Elevator() {
      elevatorMotor = new CANSparkMax(Constants.MOTOR_ID_4, MotorType.kBrushless);
      armMotor = new CANSparkMax(Constants.MOTOR_ID_5, MotorType.kBrushless);
      armEncoder = armMotor.getEncoder();
      armController = armMotor.getPIDController();
      armController.setOutputRange(-0.1, 0.1);
      armEncoder.setPositionConversionFactor(1);
      // elevatorMotor.set(preset);
    }

    public void setHeight(int preset) {
     // double current = elevatorMotor.getEncoder().getPosition();
     // double change = height[preset] - current;
    //  current = height[preset];
     // elevatorMotor.getPIDController().setReference(change, ControlType.kPosition);
     // elevatorMotor.getEncoder().setPosition(0);
      // TODO 2/18
    }

    public void setArm(int preset){
      double current = armEncoder.getPosition();
      double change = armHeights[preset] - current;
      armController.setReference(change, ControlType.kPosition);
    }

    public void moveArm(double speed){
      armMotor.set(speed);
    }

    public void resetArm(){
      armEncoder.setPosition(0);
    }

    public double getPosition(){

      return armEncoder.getPosition();

    }

    //private void configController(CANSparkMax sparkMax, double kP, double kI, double kD, double kF)
    
      
      //SparkMaxPIDController controller = sparkMax.getPIDController();
      //controller.setP(kP);
      //controller.setI(kI);
      //controller.setD(kD);
      //controller.setFF(kF);
      //controller.setOutputRange(-peakOutput, peakOutput);
      //sparkMax.setCANTimeout(100);
      //TO DO pid later
    
   
}

