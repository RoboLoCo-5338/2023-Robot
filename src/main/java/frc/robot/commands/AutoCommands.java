// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;


public class AutoCommands {

// drives a certain distance 
  public static Command driveDistanceCommand(double distance, Direction direction){
    return new FunctionalCommand(
      () -> RobotContainer.drivetrain.resetPosition(),
      () -> RobotContainer.drivetrain.driveDistance(distance, direction),
      (interrupt) -> RobotContainer.drivetrain.tankDrive(0, 0),
      () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(RobotContainer.drivetrain.targetPosition) - 0.1,
      RobotContainer.drivetrain
    );
  }

  // sets the velocity given a distance 
  public static Command driveVelocityCommand(double distance, double leftVelocity, double rightVelocity){
    return new FunctionalCommand( 
      () -> RobotContainer.drivetrain.resetVelocity(),
      () -> RobotContainer.drivetrain.tankDriveVelocity(leftVelocity, rightVelocity),
      (interrupt) -> RobotContainer.drivetrain.tankDriveVelocity(0, 0),
      () -> Math.abs(RobotContainer.drivetrain.getPosition())>= Math.abs(distance) - 0.1,
      RobotContainer.drivetrain
    );
  }

// turn command using PID
public static Command PIDTurnCommand(double angle, Direction direction){
  return new PIDCommand(
    new PIDController(0.07, 0, 0),
    () -> Math.abs(RobotContainer.drivetrain.getAngle()),
    () -> angle,
    (output) -> RobotContainer.drivetrain.tankDrive(direction==Direction.RIGHT ? -output : output, direction==Direction.RIGHT ? output : -output),
    RobotContainer.drivetrain
  );
}

// sample auto sequential command group 
  public static Command sampleAuto() {
    return new SequentialCommandGroup(
      driveDistanceCommand(20, Direction.FORWARD),
      driveVelocityCommand(20, 20, 20)
    );
  }

  // auto sequential command group for middle in community 
  public static Command middleAuto() {
    return new SequentialCommandGroup(
      // get out of the community line 
      driveVelocityCommand(10, -1, -1),
      //turn and score cube
      PIDTurnCommand(180, Direction.RIGHT),
      driveVelocityCommand(60.69, 1, 1),
      ElevatorCommands.setElevatorHeight(70), // place holder for now 
      ArmCommands.setArm(30), // place holder for now 
      EffectorCommands.effectorForward(),
      //mount charging station?
      driveVelocityCommand(76.125, 1, 1),
      driveVelocityCommand(40, 1, 1),
      driveVelocityCommand(40, -1, -1),
      driveVelocityCommand(38.0625, -1, -2)
      //engage on charging station?
    );
  }

  // sequential command group for bottom of community 
  public static Command rightAuto(){
    return new SequentialCommandGroup(
      driveVelocityCommand(10, -1, -1),
      //turn and score cube?
      driveVelocityCommand(56.75, 1, 1),
      driveVelocityCommand(40, 1, 1),
      //PIDTurnCommand(90, Direction.LEFT),
      driveVelocityCommand(22, 1, 1),
     // PIDTurnCommand(90, Direction.LEFT),
      driveVelocityCommand(38.0625, 1, 1)
      //engange on charging station?
    );
  }
}
