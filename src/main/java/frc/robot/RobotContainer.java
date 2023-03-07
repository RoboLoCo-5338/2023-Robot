// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.Direction;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ConeTipperCommands;
import frc.robot.commands.EffectorCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.LimeLight;
import frc.robot.subsystems.ConeTipper;
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
 // public static final ConeTipper coneTipper = new ConeTipper();

  public static double percent = 0.3;

  public static int coneOffset =0;

  public static int coneTipperPreset=0;

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
        // controller1.getRawAxis(1)*(percent+controller1.getRawAxis(3)*(1-percent)),
        // controller1.getRawAxis(5)*(percent+controller1.getRawAxis(3)*(1-percent))
        controller1.getRawAxis(5) * 0.3,
        controller1.getRawAxis(1) * 0.3
      ),
      drivetrain
    );

    // public Command defaultElevator = new RunCommand(//left joystick
    //   () -> m_Elevator.moveElevator(
    //    controller2.getRawAxis(1)*0.1
    //   ),
    //   m_Elevator
    // );

    // public Command defaultArm = new RunCommand(//right joystick
    //   () -> m_Elevator.moveArm(
    //    controller2.getRawAxis(5)*0.1
    //   ),
    //   m_Elevator
    // );

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

    

    // public SequentialCommandGroup defaultElev = new SequentialCommandGroup(defaultElevator, defaultArm);

    /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   
    JoystickButton forwardEffector = new JoystickButton(controller1, Constants.RBBUTTON);
    JoystickButton backwardEffector = new JoystickButton(controller1, Constants.LBBUTTON);
    JoystickButton limeLight = new JoystickButton(controller1, Constants.ABUTTON);
   

    //TEMPORARY
    JoystickButton intakeHeight = new JoystickButton(controller2, Constants.BBUTTON);
    JoystickButton bottomHeight = new JoystickButton(controller2, Constants.ABUTTON);
    intakeHeight.whileTrue(ElevatorCommands.moveElevator(0.1));
    bottomHeight.whileTrue(ElevatorCommands.moveElevator(-0.1));

    Trigger moveElevator = new Trigger(() -> Math.abs(controller2.getRawAxis(1)) > 0.1);
    Trigger moveArm = new Trigger(() -> Math.abs(controller2.getRawAxis(5)) > 0.1);
    intakeHeight.onFalse(ElevatorCommands.stopElevator());//driver
    bottomHeight.onFalse(ElevatorCommands.stopElevator());//driver


    JoystickButton mediumHeight = new JoystickButton(controller2, Constants.XBUTTON);
    JoystickButton highHeight = new JoystickButton(controller2, Constants.YBUTTON);
    mediumHeight.whileTrue(ArmCommands.moveArm(0.2));
    highHeight.whileTrue(ArmCommands.moveArm(-0.2));

    mediumHeight.onFalse(ArmCommands.stopArm());//driver
    highHeight.onFalse(ArmCommands.stopArm());//driver


    JoystickButton cubeSwitch = new JoystickButton(controller2, Constants.RBBUTTON);
    JoystickButton coneSwitch = new JoystickButton(controller2, Constants.LBBUTTON);

    Trigger forwardEffector2 = new Trigger(() -> controller2.getRawAxis(3)>0.5);
    Trigger backwardEffector2 = new Trigger(() -> controller2.getRawAxis(2)>0.5);


    JoystickButton coneTipperForward = new JoystickButton(controller2, Constants.CONE_TIPPER_FORWARD);
    JoystickButton coneTipperBackward = new JoystickButton(controller2, Constants.CONE_TIPPER_BACKWARD);

    JoystickButton coneTipperCycleUp = new JoystickButton(controller2, Constants.CONE_TIPPER_CYCLE_UP);
   

    moveElevator.whileTrue(ElevatorCommands.moveElevator(controller2.getRawAxis(1)>0 ? 0.1 : -0.1));
    moveElevator.whileFalse(ElevatorCommands.stopElevator());
    moveArm.whileTrue(ArmCommands.moveArm(controller2.getRawAxis(5)> 0 ? 0.1 : -0.1));
    moveArm.whileFalse(ArmCommands.stopArm());

    // forwardEffector.whileTrue(EffectorCommands.effectorForward());
    // backwardEffector.whileTrue(EffectorCommands.effectorReverse());

    // forwardEffector.onFalse(EffectorCommands.effectorStop());//driver
    // backwardEffector.onFalse(EffectorCommands.effectorStop());

   // limeLight.whileTrue(runLimeLight);

    

    // intakeHeight.onTrue(ElevatorCommands.setElevatorHeight(0));
    // bottomHeight.onTrue(ElevatorCommands.setElevatorHeight(1));
    // mediumHeight.onTrue(ElevatorCommands.setElevatorHeight(2+coneOffset));
    // highHeight.onTrue(ElevatorCommands.setElevatorHeight(3+coneOffset));

    // cubeSwitch.onTrue(cubeSwitchCommand);
    // coneSwitch.onTrue(coneSwitchCommand);

    forwardEffector2.whileTrue(EffectorCommands.effectorForward());
    backwardEffector2.whileTrue(EffectorCommands.effectorReverse());//operator
    forwardEffector2.onFalse(EffectorCommands.effectorStop());
    backwardEffector2.onFalse(EffectorCommands.effectorStop());//operator

    // coneTipperForward.whileTrue(ConeTipperCommands.moveForward());
    // coneTipperBackward.whileTrue(ConeTipperCommands.moveBackward());

    
    // coneTipperCycleUp.onTrue(ConeTipperCommands.setConeTipper());

  }


  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(defaultDrive);
   // m_Elevator.setDefaultCommand(defaultElev);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //REPLACE
    // return AutoCommands.moveForward(60);
  // }
}
