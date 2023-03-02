// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {}

  public static Command setElevatorHeight(int preset) {
    return new FunctionalCommand(
      () -> RobotContainer.m_Elevator.setElevatorChange(preset),
      () -> RobotContainer.m_Elevator.setElevatorHeight(),
      (interrupt) -> RobotContainer.m_Elevator.stopElevator(),
      () -> Math.abs(RobotContainer.m_Elevator.getElevatorPosition()) >= Math.abs(Elevator.elevatorChange+RobotContainer.m_Elevator.getElevatorPosition()-0.1),
      RobotContainer.m_Elevator
    );
  }
  public static Command moveElevator(Elevator elevator, double speed){
    return new InstantCommand(
      () -> RobotContainer.m_Elevator.moveElevator(speed),
      RobotContainer.m_Elevator
    );
  }

  public static Command stopElevator(Elevator elevator){
    return new InstantCommand(
      () -> RobotContainer.m_Elevator.stopElevator(),
      RobotContainer.m_Elevator
    );
  }
//todo might want to use a different method, maybe one that is timed instead of using a preset for this
  public static Command startMechanism(){
    return new SequentialCommandGroup(
      setElevatorHeight(1),
      ArmCommands.setArmPresets(RobotContainer.m_Elevator,1),
      setElevatorHeight(0)
    );
  }
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
  }