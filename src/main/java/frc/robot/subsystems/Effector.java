// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

@SuppressWarnings("Serial Warnings")
public class Effector extends SubsystemBase{

    private CANSparkMax outerMotor;
    private double speed = 0.8;
    private SparkMaxPIDController effectorController;
    //untested PID
    public static double effectorP=0.1;
    public static double effectorI=0.0;
    public static double effectorD=0.0;
    public static double effector_Forward=0.0;
    
    public Effector() {
        outerMotor = new CANSparkMax(Constants.EFFECTOR_MOTOR, MotorType.kBrushless);
        outerMotor.setIdleMode(IdleMode.kCoast);
        outerMotor.setSmartCurrentLimit(30);
       // outerMotor.setInverted(true);
        //configController();
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
     private void configController(){
    effectorController.setP(effectorP);
    effectorController.setI(effectorI);
    effectorController.setD(effectorD);
    effectorController.setFF(effector_Forward);
}

}