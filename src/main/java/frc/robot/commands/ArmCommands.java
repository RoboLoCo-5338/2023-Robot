// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Arm;


// public class ArmCommands  {

//   /** Creates a new ArmCommands. */
//   public ArmCommands() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Move arm to preset height and stop when the height is reached.
//   public static Command setArm(int preset){
//     return new FunctionalCommand(
//       () -> {},
//       () -> RobotContainer.m_Arm.setArm(preset),
//       (interrupt) -> RobotContainer.m_Arm.stopArm(),
//       () -> Math.abs(RobotContainer.m_Arm.armHeights[preset]-RobotContainer.m_Arm.getArmPosition()) <= 3,
//       RobotContainer.m_Arm);
//   }

//   // public static Command setArmAbsolute(double setpoint) {
//   //   return new PIDCommand(
//   //     new PIDController(2, 0, 0), 
//   //     () -> RobotContainer.effector.armAbsEncoder.getPosition(),
//   //     RobotContainer.m_Arm.armHeights[(int)setpoint],
//   //     output -> {RobotContainer.m_Arm.moveArm(output*2);},
//   //     //() -> Math.abs(RobotContainer.m_Arm.armHeights[preset]-RobotContainer.m_Effector.getEffectorPosition()) <= 0.1,
//   //     RobotContainer.m_Arm);
//   // }


// // command to set speed for arm
//   public static Command moveArm(double speed){
//     return new InstantCommand(
//       () -> RobotContainer.m_Arm.moveArm(speed),
//       RobotContainer.m_Arm);
//   }

//   // Command bindings for arm and elevator methods.
//   public static Command moveUp(Arm arm){
//     return new InstantCommand(
//       () -> RobotContainer.m_Arm.moveArm(0.1),
//       RobotContainer.m_Arm
//     );
//   }

//   public static Command moveDown(Arm arm){
//     return new InstantCommand(
//       () -> RobotContainer.m_Arm.moveArm(-0.1),
//       RobotContainer.m_Arm
//     );
//   }

// // stop arm from moving 
//   public static Command stopArm(){
//     return new InstantCommand(
//       () -> RobotContainer.m_Arm.stopArm(),
//       RobotContainer.m_Arm
//     );
//   }

// }
