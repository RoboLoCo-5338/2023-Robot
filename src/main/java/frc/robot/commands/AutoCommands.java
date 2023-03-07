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
  public static Command driveDistanceCommand(double distance, Direction direction){
    return new FunctionalCommand(
      () -> RobotContainer.drivetrain.resetPosition(),
      () -> RobotContainer.drivetrain.driveDistance(distance, direction),
      (interrupt) -> RobotContainer.drivetrain.tankDrive(0, 0),
      () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(RobotContainer.drivetrain.targetPosition) - 1000,
      RobotContainer.drivetrain
    );
  }
  public static Command driveVelocityCommand(double distance, double leftVelocity, double rightVelocity){
    return new FunctionalCommand( 
      () -> RobotContainer.drivetrain.resetVelocity(),
      () -> RobotContainer.drivetrain.tankDrive(leftVelocity, rightVelocity),
      (interrupt) -> RobotContainer.drivetrain.tankDriveVelocity(0, 0),
      () -> Math.abs(RobotContainer.drivetrain.getPosition())>= Math.abs(distance) - 0.1,
      RobotContainer.drivetrain
    );
  }


public static Command PIDTurnCommand(double angle, Direction direction){
  return new PIDCommand(
    new PIDController(0, 0, 0),
    () -> Math.abs(RobotContainer.drivetrain.getAngle()),
    () -> angle,
    (output) -> RobotContainer.drivetrain.tankDrive(direction==Direction.RIGHT ? -output : output, direction==Direction.RIGHT ? output : -output),
    RobotContainer.drivetrain
  );
}


  public static Command sampleAuto() {
    return new SequentialCommandGroup(
      driveDistanceCommand(20, Direction.FORWARD),
      driveVelocityCommand(20, 20, 20),
    );
  }
}
