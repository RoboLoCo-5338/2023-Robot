// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxAbsoluteEncoder;

// import frc.robot.Constants;
// import com.revrobotics.CANSparkMax.IdleMode;

// @SuppressWarnings("Serial Warnings")
// public class Effector extends SubsystemBase{

//     private CANSparkMax effectorMotor;
//     public SparkMaxAbsoluteEncoder armAbsEncoder;
//     private double speed = 0.8;
//     // private SparkMaxPIDController effectorController;
//     // private RelativeEncoder effectorEncoder;
//     //untested PID
//     public static double effectorP=0.1;
//     public static double effectorI=0.0;
//     public static double effectorD=0.0;
//     public static double effector_Forward=0.0;
    
//     public Effector() {
//         // effectorMotor = new CANSparkMax(Constants.EFFECTOR_MOTOR, MotorType.kBrushed);
//         // effectorMotor.setIdleMode(IdleMode.kBrake);
//         effectorMotor.setSmartCurrentLimit(25);
//         armAbsEncoder = effectorMotor.getAbsoluteEncoder(Type.kDutyCycle);
//         // effectorController = effectorMotor.getPIDController();
//         // effectorController.setOutputRange(-0.7, 0.7);
//         //configController();
//        // outerMotor.setInverted(true);
//         //configController();
//     }

//     public void effectorForward() {
//         effectorMotor.set(speed);
//     }

  
//     public void effectorReverse()
//     {
//         effectorMotor.set(-speed);
//     }

//     public void effectorStop() {
//         effectorMotor.set(0);
//     }

// }