// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class AutoCommands {

// drives a certain distance 
  public static Command driveDistanceCommand(double distance, Direction direction){
    return new FunctionalCommand(
      () -> RobotContainer.drivetrain.resetPosition(),
      () -> RobotContainer.drivetrain.driveDistance(distance, direction),
      (interrupt) -> RobotContainer.drivetrain.tankDrive(0, 0),
      () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(RobotContainer.drivetrain.targetPosition) - 0.1,
      RobotContainer.drivetrain
    );
  }

  // sets the velocity given a distance 
  public static Command driveVelocityCommand(double distance, double leftVelocity, double rightVelocity){
    return new FunctionalCommand( 
      () -> RobotContainer.drivetrain.resetVelocity(),
      () -> RobotContainer.drivetrain.tankDriveVelocity(-leftVelocity, -rightVelocity),
      (interrupt) -> RobotContainer.drivetrain.tankDriveVelocity(0, 0),
      () -> Math.abs(RobotContainer.drivetrain.getPosition())>= Math.abs(distance) - 0.1,
      RobotContainer.drivetrain

    );
  }

// turn command using PID
public static Command PIDTurnCommand(double angle, Direction direction){
  return new PIDCommand(
    new PIDController(0.07, 0, 0),
    () -> Math.abs(RobotContainer.drivetrain.getAngle()),
    () -> angle,
    (output) -> RobotContainer.drivetrain.tankDrive(direction==Direction.RIGHT ? -output : output, direction==Direction.RIGHT ? output : -output),
    RobotContainer.drivetrain
  );
}

// sample auto sequential command group 
  public static Command sampleAuto() {
    return new SequentialCommandGroup(
      driveDistanceCommand(20, Direction.FORWARD),
      driveVelocityCommand(20, 20, 20)
    );
  }

  public static Command scoreAndMove() {
    return new SequentialCommandGroup(
      ElevatorCommands.unStowCommand(),
      //EffectorCommands.autoEffector(-20), //not sure sign
      driveVelocityCommand(10, -1, -1),
      RobotContainer.moveMechanismPID(1),
      driveDistanceCommand(12, Direction.FORWARD),
      //EffectorCommands.autoEffector(20), //not sure sign
      driveDistanceCommand(12, Direction.BACKWARD),
      ElevatorCommands.stowCommand(),
      driveDistanceCommand(100, Direction.BACKWARD)
    );
  }

    // // sequential command group for bottom of community 
    // public static Command rightAuto(){
    //   return new SequentialCommandGroup(
    //     driveVelocityCommand(10, -1, -1),
    //     //turn and score cube
    //     scoreAndMove(),
    //     driveVelocityCommand(56.75, 1, 1),
    //     driveVelocityCommand(40, 1, 1),
    //     //PIDTurnCommand(90, Direction.LEFT),
    //     driveVelocityCommand(22, 1, 1),
    //    // PIDTurnCommand(90, Direction.LEFT),
    //     driveVelocityCommand(38.0625, 1, 1)
    //     //engange on charging station?
    //   );
    // }

  // auto sequential command group for middle in community (dock and engage)
  public static Command middleAuto() {
    return new SequentialCommandGroup(
      // get out of the community line 
      driveVelocityCommand(10, -1, -1),
      //turn and score cube
      scoreAndMove(),
      //mount charging station?
      driveVelocityCommand(76.125, 1, 1),
      driveVelocityCommand(40, 1, 1),
      driveVelocityCommand(40, -1, -1),
      driveVelocityCommand(38.0625, -1, -2)
      //engage on charging station?
    );
  }

  // move forward, unstow, preset elevator, move more forward, drop cone, move back, stow, move back (out of community line)

  // top auto sequential command group, need to calculate values for distances and actual set point (score and move out of the community line )
  public static Command leftAuto(){
    return new SequentialCommandGroup(
      driveVelocityCommand(30.345, 40, 40),
      ElevatorCommands.unStowCommand(),
      RobotContainer.moveMechanismPID(5),
      driveVelocityCommand(10, 20, 20),
      EffectorCommands.timeEffectorReverse(0), // to do
      EffectorCommands.effectorStop(),
      RobotContainer.moveMechanismPID(5),
      driveVelocityCommand(10, 20, 20),
      ElevatorCommands.stowCommand(),
      driveVelocityCommand(96.75, 40, 40)
    );
  }

  public Command RamseteTest(){
     // Create a voltage constraint to ensure we don't accelerate too fast
     DifferentialDriveVoltageConstraint autoVoltageConstraint =
     new DifferentialDriveVoltageConstraint(
         new SimpleMotorFeedforward(
             Constants.ksVolts,
             Constants.kvVoltSecondsPerMeter,
             Constants.kaVoltSecondsSquaredPerMeter),
         Constants.kDriveKinematics,
         10);

 // Create config for trajectory
 TrajectoryConfig config =
     new TrajectoryConfig(
             Constants.kMaxSpeedMetersPerSecond,
             Constants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(Constants.kDriveKinematics)
         // Apply the voltage constraint
         .addConstraint(autoVoltageConstraint);

 // An example trajectory to follow.  All units in meters.
 Trajectory trajectory =
     TrajectoryGenerator.generateTrajectory(
         // Start at the origin facing the +X direction
         new Pose2d(0, 0, new Rotation2d(0)),
         // Pass through these two interior waypoints, making an 's' curve path
         List.of(new Translation2d(1,0)),
         // End 3 meters straight ahead of where we started, facing forward
         new Pose2d(2, 0, new Rotation2d(0)),
         // Pass config
         config);

 RamseteCommand ramseteCommand =
     new RamseteCommand(
         trajectory,
         RobotContainer.drivetrain::getPose,
         new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
         new SimpleMotorFeedforward(
             Constants.ksVolts,
             Constants.kvVoltSecondsPerMeter,
             Constants.kaVoltSecondsSquaredPerMeter),
         Constants.kDriveKinematics,
         RobotContainer.drivetrain::getWheelSpeeds,
         new PIDController(Constants.kPDriveVel, 0, 0),
         new PIDController(Constants.kPDriveVel, 0, 0),
         // RamseteCommand passes volts to the callback
         RobotContainer.drivetrain::tankDriveVolts,
         RobotContainer.drivetrain);

 // Reset odometry to the starting pose of the trajectory.
 RobotContainer.drivetrain.resetOdometry(trajectory.getInitialPose());

 // Run path following command, then stop at the end.
 return ramseteCommand;
  }


}
