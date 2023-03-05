// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import frc.robot.Direction;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer; 

public final class Autos {
/* 
  public static CommandBase driveDistanceCommand(double distance, Direction direction) {
    return new FunctionalCommand(
            () -> RobotContainer.drivetrain.resetPosition(),
            () -> RobotContainer.drivetrain.driveDistance(distance, direction),
            (interrupt) -> RobotContainer.drivetrain.tankDrive(0, 0),
            () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(Drivetrain.targetPosition) - 1000,
            RobotContainer.drivetrain
    );
  }
*/

  public static CommandBase tempCommand() {
    return new RunCommand(
      () -> RobotContainer.drivetrain.tankDrive(0,0),
      RobotContainer.drivetrain
    );
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}