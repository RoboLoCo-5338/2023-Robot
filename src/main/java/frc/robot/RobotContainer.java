// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.LimeLight;
import frc.robot.subsystems.DriveSystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //public static DriveSystem driveSystem = new DriveSystem();
  // public static Climb climb = new Climb();
  public static DriveSystem driveSystem = new DriveSystem();
  public static final LimeLight limeLight = new LimeLight();

  
  private static Joystick controller1 = new Joystick(0);
  private static Joystick controller2 = new Joystick(1);
  
  // Initialize the drive command
    public Command defaultDrive = new RunCommand(
      () -> driveSystem.tankDriveVelocity(
        controller1.getRawAxis(1)*0.2,
        controller1.getRawAxis(5)*0.2
      ),
      driveSystem
    );

    public Command slowOn = new InstantCommand(
      () -> driveSystem.setSlow(true),
      driveSystem
    );

    public Command slowOff = new InstantCommand(
      () -> driveSystem.setSlow(false),
      driveSystem
    );

     public Command runLimeLight = new RunCommand(
      () -> limeLight.execute(),
      driveSystem);

    


    public Command straightTrue = new InstantCommand(
      () -> driveSystem.setStraight(true),
      driveSystem
    );

    public Command straightFalse = new InstantCommand(
      () -> driveSystem.setStraight(false),
      driveSystem
    );

  // climb percent output commands for motors
  // public static Command climbPercentForward() {
  //   return new RunCommand(
  //     () -> RobotContainer.climb.climbPercent(
  //       controller2.getRawAxis(1)
  //     ), 
  //     RobotContainer.climb
  //     );
  // }


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // drive buttons
  

    // Trigger slowToggle = new Trigger(() -> controller1.getRawAxis(3) > 0.5);
    // slowToggle.whileTrue(slowOn);
    // slowToggle.whileFalse(slowOff);

     JoystickButton limeLightButton = new JoystickButton(controller1, Constants.BBUTTON);
  

    limeLightButton.whileTrue(runLimeLight);

    

    

  }

  private void configureDefaultCommands() {
    driveSystem.setDefaultCommand(defaultDrive);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.setDefaultCommand(driveSystem, defaultDrive);


    // scheduler.addButton(() -> indexUpCommand());
    // scheduler.addButton(() -> indexDownCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
