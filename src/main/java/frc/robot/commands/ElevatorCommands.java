// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;


public class ElevatorCommands extends CommandBase {
  Elevator elevator;
  Joystick joysticks[] = new Joystick [10]();
  Constants m_constants = new Constants();

  public ElevatorCommands() {
    execute();
    elevator = new Elevator();

    for (int i = 0; i < 10; i++)
      joysticks[port: i] = new Joystick(port: i)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    for (Joystick j : enumerate Joysticks)
      if (j.getTopPressed())
        this.elevator.setHeight(0);

    if (controller3.getTopPressed()){
      this.elevator = new Elevator(Constants.height1);
    }
    if (controller4.getTopPressed()){
      Elevator elevator = new Elevator(Constants.height2);
    }
    if (controller5.getTopPressed()){
      Elevator elevator = new Elevator(Constants.height3);
    }
    if (controller6.getTopPressed()){
      Elevator elevator = new Elevator(Constants.height4);
    }
    if (controller7.getTopPressed()){
      Elevator elevator = new Elevator(Constants.height5);
    }
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
