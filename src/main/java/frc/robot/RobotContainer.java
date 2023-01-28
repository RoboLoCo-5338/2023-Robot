// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivetrain drivetrain = new Drivetrain();
  /*public static ShooterSystem shooterSystem = new ShooterSystem();
  public static Climb climb = new Climb();
  public static Intake intake = new Intake();*/
  
  private static Joystick controller1 = new Joystick(0);
  private static Joystick controller2 = new Joystick(1);
  
  // Initialize the drive command
    public Command defaultDrive = new RunCommand(
      () -> drivetrain.tankDrive(
        controller1.getRawAxis(1),
        controller1.getRawAxis(5)
      ),
      drivetrain
    );

   /* public Command slowOn = new InstantCommand(
      () -> driveSystem.setSlow(true),
      driveSystem
    );

    public Command slowOff = new InstantCommand(
      () -> driveSystem.setSlow(false),
      driveSystem
    );

    public Command straightTrue = new InstantCommand(
      () -> driveSystem.setStraight(true),
      driveSystem
    );

    public Command straightFalse = new InstantCommand(
      () -> driveSystem.setStraight(false),
      driveSystem
    );*/


    
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
     //drive buttons
    
     /* 
    // JoystickButton slowButton = new JoystickButton(controller1, Constants.STARTBUTTON);
    // slowButton.whenPressed(toggleSlow);

    // Trigger slowToggle = new Trigger(() -> controller1.getRawAxis(3) > 0.5);
    // slowToggle.whenActive(slowOn);
    // slowToggle.whenInactive(slowOff);

    // JoystickButton straightButton = new JoystickButton(controller1, Constants.XBUTTON);
    // straightButton.whileHeld(straightTrue);
    // straightButton.whenReleased(straightFalse);
    */

    // climb buttons
   /*  JoystickButton longHighPiston = new JoystickButton(controller2, Constants.LBBUTTON);
    longHighPiston.whenPressed(ClimbCommands.longHighToggle);
    
    JoystickButton longLowPiston = new JoystickButton(controller2, Constants.RBBUTTON);
    longLowPiston.whenPressed(ClimbCommands.longLowToggle);

    // shooter buttons
    JoystickButton shooterButton = new JoystickButton(controller1, Constants.BBUTTON);
    shooterButton.whileHeld(ShooterCommands.shootCommand());
    shooterButton.whenReleased(ShooterCommands.stopShootCommand());

    JoystickButton shooterOperatorButton = new JoystickButton(controller2, Constants.BBUTTON);
    shooterOperatorButton.whileHeld(ShooterCommands.shootCommand());
    shooterOperatorButton.whenReleased(ShooterCommands.stopShootCommand());*/

    // intake + index buttons
    // controller 1
    // JoystickButton intakePneumatics = new JoystickButton(controller1, Constants.YBUTTON); 
    // intakePneumatics.whenPressed(IntakeCommands.toggleIntakePneumatics());

    // controller 2
    /*JoystickButton intakePneumatics2 = new JoystickButton(controller2, Constants.YBUTTON); 
    intakePneumatics2.whenPressed(IntakeCommands.intakeDown());
    intakePneumatics2.whenReleased(IntakeCommands.intakeUp());
    
    // controller 1
    JoystickButton intakeIndexForward = new JoystickButton(controller1, Constants.RBBUTTON);
    intakeIndexForward.whenPressed(IntakeCommands.intakeIndexForward());
    intakeIndexForward.whenReleased(IntakeCommands.stopIntakeMotors());*/
    // controller 2
    // JoystickButton intakeIndexForward2 = new JoystickButton(controller2, Constants.LBBUTTON);
    // intakeIndexForward2.whenPressed(IntakeCommands.intakeIndexForward());
    // intakeIndexForward2.whenReleased(IntakeCommands.stopIntakeMotors());

    // controller 1
    //JoystickButton outakeIndexReverse = new JoystickButton(controller1, Constants.LBBUTTON);
    //outakeIndexReverse.whenPressed(IntakeCommands.outakeIndexReverse());
    //outakeIndexReverse.whenReleased(IntakeCommands.stopIntakeMotors());
    // controller 2
    // JoystickButton outakeIndexReverse2 = new JoystickButton(controller2, Constants.RBBUTTON);
    // outakeIndexReverse2.whenPressed(IntakeCommands.outakeIndexReverse());
    // outakeIndexReverse2.whenReleased(IntakeCommands.stopIntakeMotors());
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(defaultDrive);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.setDefaultCommand(drivetrain, defaultDrive);

   // scheduler.setDefaultCommand(RobotContainer.climb, winchPercent());
    // scheduler.addButton(() -> indexUpCommand());
    // scheduler.addButton(() -> indexDownCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
   // return Autos.closeAuto();
  //}
  
  
  
  
  
  
}