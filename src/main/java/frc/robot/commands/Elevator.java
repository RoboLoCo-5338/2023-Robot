// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Elevator;


public class Elevator extends Command {
  public Elevator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    target = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.Elevator.change();
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
