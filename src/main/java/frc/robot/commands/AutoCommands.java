// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;


// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.RobotContainer;

// public class AutoCommands {

// // drives a certain distance 
//   public static Command driveDistanceCommand(double distance, Direction direction){
//     return new FunctionalCommand(
//       () -> RobotContainer.drivetrain.resetPosition(),
//       () -> RobotContainer.drivetrain.driveDistance(distance, direction),
//       (interrupt) -> RobotContainer.drivetrain.tankDrive(0, 0),
//       () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(RobotContainer.drivetrain.targetPosition) - 0.1,
//       RobotContainer.drivetrain
//     );
//   }

//   // sets the velocity given a distance 
//   public static Command driveVelocityCommand(double distance, double leftVelocity, double rightVelocity){
//     return new FunctionalCommand( 
//       () -> RobotContainer.drivetrain.resetVelocity(),
//       () -> RobotContainer.drivetrain.tankDriveVelocity(-leftVelocity, -rightVelocity),
//       (interrupt) -> RobotContainer.drivetrain.tankDriveVelocity(0, 0),
//       () -> Math.abs(RobotContainer.drivetrain.getPosition())>= Math.abs(distance) - 0.1,
//       RobotContainer.drivetrain

//     );
//   }

// // turn command using PID
// public static Command PIDTurnCommand(double angle, Direction direction){
//   return new PIDCommand(
//     new PIDController(0.07, 0, 0),
//     () -> Math.abs(RobotContainer.drivetrain.getAngle()),
//     () -> angle,
//     (output) -> RobotContainer.drivetrain.tankDrive(direction==Direction.RIGHT ? -output : output, direction==Direction.RIGHT ? output : -output),
//     RobotContainer.drivetrain
//   );
// }

// // sample auto sequential command group 
//   public static Command sampleAuto() {
//     return new SequentialCommandGroup(
//       driveDistanceCommand(20, Direction.FORWARD),
//       driveVelocityCommand(20, 20, 20)
//     );
//   }

//   public static Command scoreAndMove() {
//     return new SequentialCommandGroup(
//       ElevatorCommands.unStowCommand(),
//       //EffectorCommands.autoEffector(-20), //not sure sign
//       driveVelocityCommand(10, -1, -1),
//       RobotContainer.moveMechanismPID(1),
//       driveDistanceCommand(12, Direction.FORWARD),
//       //EffectorCommands.autoEffector(20), //not sure sign
//       driveDistanceCommand(12, Direction.BACKWARD),
//       ElevatorCommands.stowCommand(),
//       driveDistanceCommand(100, Direction.BACKWARD)
//     );
//   }

//     // // sequential command group for bottom of community 
//     // public static Command rightAuto(){
//     //   return new SequentialCommandGroup(
//     //     driveVelocityCommand(10, -1, -1),
//     //     //turn and score cube
//     //     scoreAndMove(),
//     //     driveVelocityCommand(56.75, 1, 1),
//     //     driveVelocityCommand(40, 1, 1),
//     //     //PIDTurnCommand(90, Direction.LEFT),
//     //     driveVelocityCommand(22, 1, 1),
//     //    // PIDTurnCommand(90, Direction.LEFT),
//     //     driveVelocityCommand(38.0625, 1, 1)
//     //     //engange on charging station?
//     //   );
//     // }

//   // auto sequential command group for middle in community (dock and engage)
//   public static Command middleAuto() {
//     return new SequentialCommandGroup(
//       // get out of the community line 
//       driveVelocityCommand(10, -1, -1),
//       //turn and score cube
//       scoreAndMove(),
//       //mount charging station?
//       driveVelocityCommand(76.125, 1, 1),
//       driveVelocityCommand(40, 1, 1),
//       driveVelocityCommand(40, -1, -1),
//       driveVelocityCommand(38.0625, -1, -2)
//       //engage on charging station?
//     );
//   }

//   // move forward, unstow, preset elevator, move more forward, drop cone, move back, stow, move back (out of community line)

//   // top auto sequential command group, need to calculate values for distances and actual set point (score and move out of the community line )
//   public static Command leftAuto(){
//     return new SequentialCommandGroup(
//       EffectorCommands.timeEffectorReverse(300),
//       ElevatorCommands.autoUnStowCommand(),
//       EffectorCommands.timeEffectorReverse(300),
//       new WaitCommand(0.05),
//       RobotContainer.moveMechanismPID(5),
//       EffectorCommands.timeEffectorReverse(300),
//       new WaitCommand(0.05),
//       driveDistanceCommand(18, Direction.FORWARD),
//       EffectorCommands.timeEffectorForward(300),
//       new WaitCommand(0.05),
//       driveDistanceCommand(18, Direction.BACKWARD),
//       RobotContainer.moveMechanismPID(0),
//       driveDistanceCommand(120, Direction.BACKWARD)
//     );
//   }

//   public static Command engageAndScore(){
//     return new SequentialCommandGroup(
//       EffectorCommands.timeEffectorReverse(500),
//       ElevatorCommands.unStowCommand(),
//       EffectorCommands.timeEffectorReverse(500),
//       new WaitCommand(0.2),
//       RobotContainer.moveMechanismPID(5),
//       EffectorCommands.timeEffectorReverse(500),
//       new WaitCommand(0.2),
//       driveDistanceCommand(18, Direction.FORWARD),
//       EffectorCommands.timeEffectorForward(500),
//       new WaitCommand(0.2),
//       driveDistanceCommand(18, Direction.BACKWARD),
//       ElevatorCommands.stowCommand(),
//       driveDistanceCommand(75, Direction.BACKWARD)
//     );
//   }
//   public static Command scoreAuto(){
//     return new SequentialCommandGroup(
//       driveDistanceCommand(24, Direction.BACKWARD),
//       EffectorCommands.timeEffectorReverse(300),
//       ElevatorCommands.autoUnStowCommand(),
//       EffectorCommands.timeEffectorReverse(300),
//       new WaitCommand(0.05),
//       RobotContainer.moveMechanismPID(5),
//       EffectorCommands.timeEffectorReverse(300),
//       new WaitCommand(0.05),
//       EffectorCommands.timeEffectorForward(300),
//       new WaitCommand(0.05),
//       RobotContainer.moveMechanismPID(0),
//       driveDistanceCommand(120, Direction.BACKWARD)
//     );
//   }
//   // negative --> down, using roll 

//   // public static Command scoreandEngage(){
//   //   return new SequentialCommandGroup(
//   //     EffectorCommands.timeEffectorReverse(1000),
//   //     driveDistanceCommand(10, Direction.FORWARD),
//   //     ElevatorCommands.unStowCommand(),
//   //     EffectorCommands.timeEffectorReverse(1000),
//   //     new WaitCommand(0.5),
//   //     RobotContainer.moveMechanismPID(1),
//   //     new WaitCommand(0.5),
//   //     EffectorCommands.timeEffectorForward(1000),
//   //     EffectorCommands.effectorStop()


//   //   );
//   //   }
    
//   }
