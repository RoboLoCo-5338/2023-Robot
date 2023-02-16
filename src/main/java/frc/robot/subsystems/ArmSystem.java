// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.Direction;
import frc.robot.Robot;
import frc.robot.RobotContainer; 




public class ArmSystem extends SubsystemBase
{

    private CANSparkMax outerMotor;
    
   
    public void effectorSystem()
    {
        outerMotor = new CANSparkMax(Constants.MOTOR_ID_1, MotorType.kBrushless);
        this.outerMotor.setInverted(true);
        /* this.outerMotor.setStatusFramePeriod(StatusFrame_2_Feedback0, 200); */
       
       /* effectorSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREeffectorSolenoidPCM,Constants.INTAKE_PISTON_ID[0],Constants.INTAKE_PISTON_ID[1]);*/ 
        
    }


    public void effectorIntake () 
    {
     outerMotor.set(0.5);
    }

    public void effectorOutake ()
    {
        outerMotor.set(-0.5);
    }

}

