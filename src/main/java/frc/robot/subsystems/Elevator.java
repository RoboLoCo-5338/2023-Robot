// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

@SuppressWarnings("Serial Warnings")
public class Elevator extends SubsystemBase {
  private CANSparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private SparkMaxPIDController elevatorController;
  public double[] elevatorHeights = {0,0,-44};
  public static double elevatorChange=0;
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  public double[] armHeights = {50,100,91};
  public static double armChange = 0;

  //untested PID
  public static double elevatorP=0.1;
  public static double elevatorI=0.0;
  public static double elevatorD=0.0;
  public static double elevator_Forward=0.0;

  public static double armP=0.1;
  public static double armI=0.0;
  public static double armD=0.0;
  public static double armFeed_Forward=0.0;

  public Elevator() {
      elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR, MotorType.kBrushless);
      elevatorMotor.setIdleMode(IdleMode.kBrake);
      elevatorEncoder = elevatorMotor.getEncoder();
      elevatorController = elevatorMotor.getPIDController();
      elevatorController.setOutputRange(-0.2, 0.2);
      //elevatorEncoder.setPositionConversionFactor(1);
      elevatorMotor.setSmartCurrentLimit(40);
      armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
      armMotor.setIdleMode(IdleMode.kBrake);
      armEncoder = armMotor.getEncoder();
      armController = armMotor.getPIDController();
      armController.setOutputRange(-0.2, 0.2);
      //armEncoder.setPositionConversionFactor(1);
      armMotor.setSmartCurrentLimit(40);

      configController();
      
    }
  
  
  
    public void setElevatorHeight(int preset){
      
      elevatorController.setReference(elevatorHeights[preset],  CANSparkMax.ControlType.kPosition);

    }
    
  
    public void moveElevator(double speed){
      SmartDashboard.putNumber("Elevator Position teleop", getElevatorPosition());
      elevatorMotor.set(speed);
    }
    public void resetElevator(){
      elevatorEncoder.setPosition(0);
    }
    public void setArmChange(int preset){
      double current = armEncoder.getPosition();
      SmartDashboard.putNumber("Arm Preset", preset);
      SmartDashboard.putNumber("Arm position arm change", armEncoder.getPosition());
      armChange = armHeights[preset] - current;
      SmartDashboard.putNumber("Arm change", armChange);
    }

    public void setArm(int preset){
      armController.setReference(armHeights[preset], CANSparkMax.ControlType.kPosition);
    }
  
    public void moveArm(double speed){
      SmartDashboard.putNumber("Arm Position Teleop", getArmPosition());
      armMotor.set(speed);
    }

    public void resetArm(){
      armEncoder.setPosition(0);
    }

    public double getArmPosition(){

      return armEncoder.getPosition();
    

    }

    public void stopArm(){  
      armMotor.set(0);
    }

    public void stopElevator(){
      elevatorMotor.set(0);
    }

    public double getElevatorPosition(){

      return elevatorEncoder.getPosition();

    }
    
    private void configController(){
    
    // PID config for the elevator
    elevatorController.setP(elevatorP);
    elevatorController.setI(elevatorI);
    elevatorController.setD(elevatorD);
    elevatorController.setFF(elevator_Forward);

    // PID config for the arm
    armController.setP(armP);
    armController.setI(armI);
    armController.setD(armD);
    armController.setFF(armFeed_Forward);
  }
}