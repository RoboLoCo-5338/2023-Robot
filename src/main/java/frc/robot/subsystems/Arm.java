// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;


/** Add your docs here. */
public class Arm extends SubsystemBase{
    private CANSparkMax armMotor;

    public Arm() {
        armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
    }

   

}  