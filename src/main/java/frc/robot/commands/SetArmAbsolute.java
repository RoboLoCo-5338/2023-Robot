// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmAbsolute extends PIDCommand {
  private int setPoint;

  /** Creates a new SetArmAbsolute. */
  public SetArmAbsolute(int setpoint) {
    super(
        // The controller that the command will use
        new PIDController(2, 0, 0),
        // This should return the measurement
        () -> RobotContainer.effector.armAbsEncoder.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> RobotContainer.m_Arm.armHeights[setpoint],
        // This uses the output
        output -> {{RobotContainer.m_Arm.moveArm(output*2);}
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
        setPoint = setpoint;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.m_Arm.armHeights[setPoint]-RobotContainer.effector.armAbsEncoder.getPosition()) <= 0.1);
  }
}
