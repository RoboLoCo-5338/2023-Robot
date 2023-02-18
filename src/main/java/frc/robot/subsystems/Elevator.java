// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevatorMotor;

  public Elevator(int preset) {
      elevatorMotor = new CANSparkMax(Constants.MOTOR_ID_4, MotorType.kBrushless);
      elevatorMotor.set(preset);
    }

    public void setHeight(int h) {
      elevatorMotor.set(g);
    }
}

