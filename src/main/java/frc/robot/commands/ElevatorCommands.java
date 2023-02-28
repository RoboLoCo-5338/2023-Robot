// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;


public class ElevatorCommands extends CommandBase {

  public ElevatorCommands() {
    execute();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Bindings for elevator methods.
  
  public void setElevator(Elevator elevator, int preset){
    elevator.setElevatorHeight(preset);
  }

  public void moveUp(Elevator elevator){
    elevator.moveElevator(0.1);
  }

  public void moveDown(Elevator elevator){
    elevator.moveElevator(-0.1);
  }

  public void stop(Elevator elevator){
    elevator.stopElevator();
  }




  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
