// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.RobotContainer;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// @SuppressWarnings("Serial Warnings")
// public class ElevatorCommands {
//   public ElevatorCommands() {
    
//   }
  
//   public static Command setElevatorHeight(int preset) {
//     return new FunctionalCommand(
//       () -> {},
//       () -> RobotContainer.m_Elevator.setElevatorHeight(preset),
//       (interrupt) -> RobotContainer.m_Elevator.stopElevator(),
//       () ->  Math.abs(RobotContainer.m_Elevator.elevatorHeights[preset]-RobotContainer.m_Elevator.getElevatorPosition())<=3,
//       RobotContainer.m_Elevator
//     );
//   }
  

  

//   public static Command moveElevator(double speed){
//     return new InstantCommand(
//       () -> RobotContainer.m_Elevator.moveElevator(speed),
//       RobotContainer.m_Elevator
//     );
//   }
  
//   public static Command moveUpElevator(){
//     return new InstantCommand(
//       () -> RobotContainer.m_Elevator.moveElevator(0.1),
//       RobotContainer.m_Elevator
//     );
//   }
  
//   public static Command moveDownElevator(){
//     return new InstantCommand(
//       () -> RobotContainer.m_Elevator.moveElevator(-0.1),
//       RobotContainer.m_Elevator
//     );
//   }

//   public static Command stopElevator(){
//     return new InstantCommand(
//       () -> RobotContainer.m_Elevator.stopElevator(),
//       RobotContainer.m_Elevator
//     );
//   }

  
//   //todo might want to use a different method, maybe one that is timed instead of using a preset for this
//   public static Command unStowCommand(){
//     return new SequentialCommandGroup(
//       ArmCommands.setArm(2), //kickstand
//       setElevatorHeight(2), //elevator up
//       ArmCommands.setArm(3), //arm out
//       setElevatorHeight(4)
//     );
//   }

//   public static Command autoUnStowCommand(){
//     return new SequentialCommandGroup(
//       ArmCommands.setArm(2), //kickstand
//       setElevatorHeight(2), //elevator up
//       ArmCommands.setArm(3) //arm out
//     );
//   }
  
//   public static Command stowCommand(){
//     return new SequentialCommandGroup(
//       setElevatorHeight(3), //elevator up
//       ArmCommands.setArm(4), //arm in
//       setElevatorHeight(4) //elevator down
//     );
//   }
// }
