package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;

public class EffectorCommands {
  
    public static Command effectorReverse() {
      return new InstantCommand(
        () -> RobotContainer.effector.effectorReverse(),
        RobotContainer.effector
      );
    }
    
    public static Command effectorForward() {
      return new InstantCommand(
        () -> RobotContainer.effector.effectorForward(),
        RobotContainer.effector
      );
    }

    public static Command effectorStop() {
      return new InstantCommand(
        () -> RobotContainer.effector.effectorStop(),
        RobotContainer.effector
      );
    }

    public static Command autoEffector(int setpoint) {
      return new FunctionalCommand(
        () -> {},
        () -> RobotContainer.effector.setEffectorRef(setpoint),
        (interrupt) -> RobotContainer.effector.effectorStop(),
        () ->  Math.abs(setpoint-RobotContainer.effector.getEffectorPosition())<=1,
        RobotContainer.effector
      );
    }

    // public static Command timeEffector(double time) {
    //   double start=getMatchTime
    //   return new FunctionalCommand(
    // }
  }