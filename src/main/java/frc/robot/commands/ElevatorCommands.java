// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class ElevatorCommands extends CommandBase {
  private Elevator m_Elevator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCommands(Elevator Elevator) {
    m_Elevator = Elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     Constants m_constants = new Constants();

    Joystick controller3 = new Joystick(3);
    Joystick controller4 = new Joystick(4);
    Joystick controller5 = new Joystick(5);
    Joystick controller6 = new Joystick(6);
    Joystick controller7 = new Joystick(7);


    if (controller3.getTopPressed()){
      Elevator elevator = new Elevator(Constants.height1);
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
