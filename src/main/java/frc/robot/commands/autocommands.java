// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 /** Creates a new autocommands. */
// Use addRequirements() here to declare subsystem dependencies.
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;


public class autocommands {

      /** moveback, turn angle, extend arm, moveforward (out of community)*/

      //moveback 
      public static Command moveBackward(double distance){
        return new FunctionalCommand(
          () -> RobotContainer.drivetrain.resetPosition(),
          () -> RobotContainer.drivetrain.driveDistance(distance , Direction.BACKWARD),
          (interrupt) -> RobotContainer.drivetrain.tankDriveVelocity(0, 0),
          () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(Drivetrain.targetPosition) - 1000,
          RobotContainer.drivetrain
        );
      }

    //turns the robot a certain amount to face the scoring area 

    public static Command turnAngle(double angle, Direction direction){
      return new FunctionalCommand(
        () -> RobotContainer.drivetrain.resetAngle(),
        () -> RobotContainer.drivetrain.angleTurn(direction),
        (interrupt) -> RobotContainer.drivetrain.tankDriveVelocity(0, 0),
        () -> Math.abs(RobotContainer.drivetrain.getAngle()) >= angle,
        RobotContainer.drivetrain
      );
    }

    // moves forward 
    public static Command moveForward(double distance){
      return new FunctionalCommand(
        () -> RobotContainer.drivetrain.resetPosition(),
        () -> RobotContainer.drivetrain.driveDistance(distance, Direction.FORWARD),
        (interrupt) -> RobotContainer.drivetrain.tankDriveVelocity(0, 0),
        () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(Drivetrain.targetPosition) - 1000,
        RobotContainer.drivetrain
      );
    }
    
    }
