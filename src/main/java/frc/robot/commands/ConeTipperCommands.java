package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConeTipper;

public class ConeTipperCommands {



    // public static Command setConeTipper(){
    //     return new FunctionalCommand(
    //         () -> RobotContainer.coneTipper.setConeChange(RobotContainer.m_Elevator),
    //         () -> RobotContainer.coneTipper.setPosition(),
    //         (interrupt) -> RobotContainer.coneTipper.stopConeTipper(),
    //         () -> Math.abs(RobotContainer.coneTipper.getConeTipperPosition())  >= Math.abs(ConeTipper.coneChange+RobotContainer.coneTipper.getConeTipperPosition()-0.1),
    //         RobotContainer.coneTipper
    //         );
    // }

    // public static Command moveForward(){
    //     return new InstantCommand(
    //         () -> RobotContainer.coneTipper.moveConeTipper(0.1),
    //         RobotContainer.coneTipper
    //     );
    // }

    // public static Command moveBackward(){
    //     return new InstantCommand(
    //         () -> RobotContainer.coneTipper.moveConeTipper(-0.1),
    //         RobotContainer.coneTipper
    //     );
    // }

    // public static Command stopConeTipper(){
    //     return new InstantCommand(
    //         () -> RobotContainer.coneTipper.stopConeTipper(),
    //         RobotContainer.coneTipper
    //     );
    // }




}
