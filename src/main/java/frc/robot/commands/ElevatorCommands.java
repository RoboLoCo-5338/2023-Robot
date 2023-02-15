// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Elevator;


public class ElevatorCommands extends CommandBase {

  public ElevatorCommands() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    public static CommandBase change = new InstantCommand(
      () -> RobotContainer.Elevator.change(),
      RobotContainer.Elevator


  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    frc.robot.RobotContainer.Elevator.change();
  }

  @Override
  public void end(boolean interrupted) {
  }


  // Called once after isFinished returns true
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - target > targetTime;
  }
}
// slightly changed from 2022 shooter code
