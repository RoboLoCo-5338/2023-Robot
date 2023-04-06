// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED;


public class LEDCommands  {

  /** Creates a new ArmCommands. */
  public LEDCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  //LED rainbow hopefully
  public static Command party() {
      return new RunCommand(
        () -> RobotContainer.led.rainbow(),
        RobotContainer.led
      );
    }


}
