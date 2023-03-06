// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class ArmCommands {


//todo the elevator parameter might not be needed
  public static Command setArmPresets(Elevator elevator, int preset){

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

  // Move arm to preset height and stop when the height is reached.
  public static Command setArm(Elevator elevator, int preset){
    return new FunctionalCommand(
      () -> RobotContainer.m_Elevator.setArmChange(preset),
      () -> RobotContainer.m_Elevator.setArm(),
      (interrupt) -> RobotContainer.m_Elevator.stopArm(),
      () -> Math.abs(RobotContainer.m_Elevator.getArmPosition()) >= Math.abs(Elevator.armChange+RobotContainer.m_Elevator.getArmPosition()-0.1),
      RobotContainer.m_Elevator);
  }


  public static Command moveArm(double speed){
    return new InstantCommand(
      () -> RobotContainer.m_Elevator.moveArm(speed),
      RobotContainer.m_Elevator);
  }

  // Command bindings for arm and elevator methods.
  public static Command moveUp(Elevator elevator){
    return new InstantCommand(
      () -> RobotContainer.m_Elevator.moveArm(0.1),
      RobotContainer.m_Elevator
    );
  }

  public static Command moveDown(Elevator elevator){
    return new InstantCommand(
      () -> RobotContainer.m_Elevator.moveArm(-0.1),
      RobotContainer.m_Elevator
    );
  }

  public static Command stopArm(){
    return new InstantCommand(
      () -> RobotContainer.m_Elevator.stopArm(),
      RobotContainer.m_Elevator
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
