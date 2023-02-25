// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ArmCommands extends CommandBase {
  /** Creates a new ArmCommands. */
  public ArmCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public void setArm(Elevator elevator, int preset){
    elevator.setArm(preset);
  }

  public void moveUp(Elevator elevator){
    elevator.moveArm(0.1);
  }

  public void moveDown(Elevator elevator){
    elevator.moveArm(-0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
