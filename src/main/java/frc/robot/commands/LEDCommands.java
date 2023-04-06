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


    public static Command reverse(){
      return new RunCommand(() -> RobotContainer.led.reverse(), RobotContainer.led);
    }

    public static Command update(){
      return new RunCommand(() -> RobotContainer.led.update(), RobotContainer.led);
    }

    public static Command teamColors(){
      return new RunCommand(() -> RobotContainer.led.teamColors(),RobotContainer.led);
    }

  
 



}