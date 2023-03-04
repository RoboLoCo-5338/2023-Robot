// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.EffectorCommands;
import frc.robot.commands.EffectorCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.LimeLight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  public static final Elevator m_Elevator = new Elevator();
  public static ElevatorCommands m_ElevatorCommands;
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final LimeLight LimeLight = new LimeLight();
  public static final Effector effector = new Effector();

  public static int coneOffset =0;

  // controllers
  private static Joystick controller1 = new Joystick(0); //driver
  private static Joystick controller2 = new Joystick(1); //operator

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  public Command defaultDrive = new RunCommand(
      () -> drivetrain.tankDrive(
        controller1.getRawAxis(1),
        controller1.getRawAxis(5)
      ),
      drivetrain
    );

    public Command defaultElevator = new RunCommand(//left joystick
      () -> m_Elevator.moveElevator(
       controller2.getRawAxis(1)*0.1
      ),
      m_Elevator
    );

    public Command defaultArm = new RunCommand(//right joystick
      () -> m_Elevator.moveArm(
       controller2.getRawAxis(5)*0.1
      ),
      m_Elevator
    );

    public Command coneSwitchCommand = new InstantCommand(
      () -> {coneOffset=2;}
    );

    public Command cubeSwitchCommand = new InstantCommand(
      () -> {coneOffset=0;}//starting value for cube indexing-ish?
    );

    public Command runLimeLight = new InstantCommand(
      () -> LimeLight.execute());

    public Command swapPipeline = new InstantCommand(
    () -> LimeLight.setPipeline());
    

    public SequentialCommandGroup defaultElev = new SequentialCommandGroup(defaultElevator, defaultArm);


    /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  // private Command configureBindings(
  //   () -> EffectorCommands.effectorForward(
  //     controller1.getButton(7),
  //   () -> EffectorCommands.effectorReverse(
  //     controller1.getButton(8), 
  //     EffectorCommands
  //   ));

    // Tried to do Command but error only went away when I used @ sign. I do not know why
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // START HERE THURSDAY 
    

  // }
private void configureBindings(){
  JoystickButton effectorReverseButton2 = new JoystickButton(controller2, Constants.BACKBUTTON);
  effectorReverseButton2.whileTrue(EffectorCommands.effectorReverse() );

  JoystickButton effectorForwardButton2 = new JoystickButton(controller2, Constants.STARTBUTTON);
  effectorForwardButton2.whileTrue(EffectorCommands.effectorForward() );
 
  JoystickButton effectorReverseButton1 = new JoystickButton(controller1, Constants.LBBUTTON);
  effectorReverseButton1.whileTrue(EffectorCommands.effectorReverse() );
  effectorReverseButton1.whileFalse(EffectorCommands.effectorStop () );

  JoystickButton effectorForwardButton1 = new JoystickButton(controller1, Constants.RBBUTTON);
  effectorForwardButton1.whileTrue(EffectorCommands.effectorForward() );
  effectorForwardButton1.whileFalse(EffectorCommands.effectorStop () );


}

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(defaultDrive);
  } 

  
    
  
    
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.tempCommand();
  }
}