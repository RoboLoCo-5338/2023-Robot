package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConeTipper;

public class ConeTipperCommands extends CommandBase {


    public ConeTipperCommands() {
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


    public static Command setConeTipper(){
        return new FunctionalCommand(
            () -> RobotContainer.coneTipper.setConeChange(RobotContainer.m_Elevator),
            () -> RobotContainer.coneTipper.setPosition(),
            (interrupt) -> RobotContainer.coneTipper.stopConeTipper(),
            () -> Math.abs(RobotContainer.coneTipper.getConeTipperPosition())  >= Math.abs(ConeTipper.coneChange+RobotContainer.coneTipper.getConeTipperPosition()-0.1),
            RobotContainer.coneTipper
            );
    }

    public static Command moveForward(){
        return new InstantCommand(
            () -> RobotContainer.coneTipper.moveConeTipper(0.1),
            RobotContainer.coneTipper
        );
    }

    public static Command moveBackward(){
        return new InstantCommand(
            () -> RobotContainer.coneTipper.moveConeTipper(-0.1),
            RobotContainer.coneTipper
        );
    }

    public static Command stopConeTipper(){
        return new InstantCommand(
            () -> RobotContainer.coneTipper.stopConeTipper(),
            RobotContainer.coneTipper
        );
    }




}
