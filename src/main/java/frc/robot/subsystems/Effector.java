// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;


public class Effector extends SubsystemBase{

    private CANSparkMax outerMotor;
    private double speed = 0.5;
   
    public void effectorSystem() {
        outerMotor = new CANSparkMax(Constants.MOTOR_ID_1, MotorType.kBrushless);
        this.outerMotor.setInverted(true);
        
    }


    public void effectorForward() {
        outerMotor.set(speed);
    }

    public void effectorReverse()
    {
        outerMotor.set(-speed);
    }

    public void effectorStop() {
        outerMotor.set(0);
    }

}

