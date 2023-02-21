package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Elevator;

/**
 *
 */
public class PIDCommand1 extends PIDCommand {
        public PIDCommand1(double setpoint, Elevator subsystem) {
                // super(new PIDController(1.0, 0.0, 0.0), subsystem::getPIDCommand_Input,
                // setpoint , output -> subsystem.setThisPID(output));
                super(new PIDController(1.0, 0.0, 0.0),
                                subsystem::getPIDCommand_Input, setpoint,
                                output -> subsystem.setPIDCommand_Output(output));
                getController().setTolerance(0.2);
        }

        @Override
        public boolean isFinished() {
                // End when the controller is at the reference.
                return getController().atSetpoint();
        }
}
