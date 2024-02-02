// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.commands.ArmCommands;
// import frc.robot.commands.AutoCommands;
// import frc.robot.commands.EffectorCommands;
// import frc.robot.commands.ElevatorCommands;
//import frc.robot.commands.LimeLight;
// import frc.robot.commands.SetArmAbsolute;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Effector;
// // import frc.robot.subsystems.Elevator;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  // The robot's subsystems and commands are defined here... + percent and coneOffset
  // public static final Elevator m_Elevator = new Elevator();
  // public static final Arm m_Arm = new Arm();
  // public static ElevatorCommands m_ElevatorCommands;
  public static final Drivetrain drivetrain = new Drivetrain();
  //public static final LimeLight LimeLight = new LimeLight();
  // public static final Effector effector = new Effector();

  public static AHRS navX = new AHRS(SPI.Port.kMXP);
  public static double percent = 0.3;
  public static int coneOffset = 0;

  public static int reverseModifier=1;
  private static double speedMod=0; //not sure what this should be?? 

  // controllers
  public static Joystick controller1 = new Joystick(0); //driver
  public static Joystick controller2 = new Joystick(1); //operator

//   private static XboxController controller3 = new XboxController(0); potential driver controller stuff
//   private static XboxController controller4 = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }


// Drive using the joysticks

  public Command defaultDrive = new RunCommand(
    () -> {
      if(reverseModifier<0){
        drivetrain.tankDrive(
          // controller1.getRawAxis(1)*(percent+controller1.getRawAxis(3)*(1-percent)),
          // controller1.getRawAxis(5)*(percent+controller1.getRawAxis(3)*(1-percent))
          (controller1.getRawAxis(1)+Math.signum(controller1.getRawAxis(1))*speedMod) * 0.4*reverseModifier,
          (controller1.getRawAxis(5)+Math.signum(controller1.getRawAxis(5))*speedMod) * 0.4*reverseModifier
        );
      }else{
        drivetrain.tankDrive(
          // controller1.getRawAxis(1)*(percent+controller1.getRawAxis(3)*(1-percent)),
          // controller1.getRawAxis(5)*(percent+controller1.getRawAxis(3)*(1-percent))
          (controller1.getRawAxis(5)+Math.signum(controller1.getRawAxis(5))*speedMod) * 0.4*reverseModifier,
          (controller1.getRawAxis(1)+Math.signum(controller1.getRawAxis(1))*speedMod) * 0.4*reverseModifier
        );
      }
     },
    drivetrain
  );

    public Command coneSwitchCommand = new InstantCommand(//switches to the cone heights (indexes for the list)
      () -> {coneOffset=2;}
    );

    public Command cubeSwitchCommand = new InstantCommand(//switches to the cube heights (indexes for the list)
      () -> {coneOffset=0;}//starting value for cube indexing-ish?
    );

    //public Command runLimeLight = new InstantCommand(//runs limelight code
    //  () -> LimeLight.execute());

    //public Command swapPipeline = new InstantCommand(//changes Limelight
    //() -> LimeLight.setPipeline());

    
    public Command reverse = new InstantCommand(
      () -> { reverseModifier*=-1;}
   );
 
   public Command speedBoost = new RunCommand(
     () -> {speedMod=controller1.getRawAxis(3)*1.0;}
   );

   public Command speedOff = new InstantCommand(
     () -> {speedMod=0;}
   );

   public static ParallelCommandGroup moveMechanismPID(int preset){
    return new ParallelCommandGroup(
      // ElevatorCommands.setElevatorHeight(preset), 
      // new SetArmAbsolute(preset)
     );
   }


    /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public static JoystickButton limeLight;
  private void configureBindings() {
    //variables 
    JoystickButton forwardEffector = new JoystickButton(controller1, Constants.RBBUTTON);
    JoystickButton backwardEffector = new JoystickButton(controller1, Constants.LBBUTTON);
    limeLight = new JoystickButton(controller1, Constants.ABUTTON);
    
    //presets
    JoystickButton intakeHeight = new JoystickButton(controller2, Constants.BBUTTON);
    // JoystickButton bottomHeight = new JoystickButton(controller2, Constants.ABUTTON);
    JoystickButton mediumHeight = new JoystickButton(controller2, Constants.XBUTTON);
    JoystickButton highHeight = new JoystickButton(controller2, Constants.YBUTTON);

    //stow buttons
    JoystickButton stow = new JoystickButton(controller2, Constants.LBBUTTON);
    JoystickButton unstow = new JoystickButton(controller2, Constants.RBBUTTON);

    // JoystickButton cubeSwitch = new JoystickButton(controller2, Constants.RBBUTTON);
    // JoystickButton coneSwitch = new JoystickButton(controller2, Constants.LBBUTTON);

    Trigger forwardEffector2 = new Trigger(() -> controller2.getRawAxis(3)>0.5);
    Trigger backwardEffector2 = new Trigger(() -> controller2.getRawAxis(2)>0.5);

    Trigger moveElevatorUp = new Trigger(() -> controller2.getRawAxis(1) > 0.1);//checks whether joystick is either greater than 0.1 or less that -0.1
    //Trigger moveArm = new Trigger(() -> Math.abs(controller2.getRawAxis(5)) > 0.1);
    Trigger moveArmUp = new Trigger(() -> controller2.getRawAxis(5) > 0.1 );
    Trigger moveArmDown = new Trigger(() ->  controller2.getRawAxis(5)< -0.1);

   Trigger moveElevatorDown  = new Trigger(() ->  controller2.getRawAxis(1) < -0.1);
   Trigger revTrigger = new  Trigger(() -> controller1.getRawAxis(2)>0.5);

   Trigger speed = new Trigger(() -> controller1.getRawAxis(3)>0.1);

    speed.whileTrue(speedBoost);


    revTrigger.onTrue(reverse);
    speed.onFalse(speedOff);
   
    //operator presets   
    // bottomHeight.onTrue(moveMechanismPID(0));
    mediumHeight.onTrue(moveMechanismPID(1));
    //mediumHeight.onTrue(ArmCommands.setArmAbsolute(0.5));
    highHeight.onTrue(moveMechanismPID(5)); //ADD PRESETS
    // intakeHeight.onTrue(moveMechanismPID(6)); //ADD PRESETS

    // unstow.onTrue(ElevatorCommands.unStowCommand());
    // stow.onTrue(ElevatorCommands.stowCommand());

    // moveElevatorUp.whileTrue(ElevatorCommands.moveElevator( 0.4 ));
    // moveElevatorDown.whileTrue(ElevatorCommands.moveElevator(-0.4));
    // moveElevatorDown.whileFalse(ElevatorCommands.stopElevator());
    // moveElevatorUp.whileFalse(ElevatorCommands.stopElevator());
    // moveArmUp.whileTrue(ArmCommands.moveArm(-0.4));
    // moveArmDown.whileTrue(ArmCommands.moveArm(0.4));
    // moveArmDown.whileFalse(ArmCommands.stopArm());
    // moveArmUp.whileFalse(ArmCommands.stopArm());

    // //operator
    // forwardEffector2.whileTrue(EffectorCommands.effectorForward());
    // backwardEffector2.whileTrue(EffectorCommands.effectorReverse());
    // forwardEffector2.onFalse(EffectorCommands.effectorStop());
    // backwardEffector2.onFalse(EffectorCommands.effectorStop());

    // //driver
    // forwardEffector.whileTrue(EffectorCommands.effectorForward());
    // backwardEffector.whileTrue(EffectorCommands.effectorReverse());
    // forwardEffector.onFalse(EffectorCommands.effectorStop());
    // backwardEffector.onFalse(EffectorCommands.effectorStop());
  }


  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(defaultDrive);
  }

  public Command getAutonomousCommand(){
    return null;
    // return AutoCommands.leftAuto();
    //return AutoCommands.scoreAndMove();

    //return AutoCommands.engageAndScore();
  }
}
