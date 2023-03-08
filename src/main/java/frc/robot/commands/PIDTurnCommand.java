// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

public class PIDTurnCommand extends PIDCommand {
    private double angle = 0;
    /** Creates a new PIDTurnCommand. */
    public PIDTurnCommand(double angle, Direction direction) {
      super(
          // The controller that the command will use
          new PIDController(
                  0.022,
                  0.0000005,
                  0.0),
          // This should return the measurement
          () -> Math.abs(RobotContainer.drivetrain.getAngle()),
          // This should return the setpoint (can also be a constant)
          () -> angle,
          // This uses the output
          // Use the output here
          (output) -> RobotContainer.drivetrain.tankDriveVelocity((direction==Direction.LEFT ? 1 : -1)*output, (direction==Direction.LEFT ? 1 : -1)*-output)
      );
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(RobotContainer.drivetrain);
      // Configure additional PID options by calling `getController` here.
      this.angle = angle;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return getController().atSetpoint() || ((Math.abs(RobotContainer.drivetrain.getAngle())) > (Math.abs(this.angle) - 2.5));
    }
}
