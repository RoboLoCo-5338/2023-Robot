package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    

    /** Creates a new DriveSystem. */
  private static WPI_TalonFX rightFront;
  private static WPI_TalonFX rightRear;
  private static WPI_TalonFX leftFront;
  private static WPI_TalonFX leftRear;


  private static double PEAK_OUTPUT = 0.2;
  private static final int DEFAULT_TIMEOUT = 30;

  public DriveSubsystem(){

    rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT_DRIVE);
    rightRear = new WPI_TalonFX(Constants.RIGHT_REAR_DRIVE);
    leftFront = new WPI_TalonFX(Constants.LEFT_FRONT_DRIVE);
    leftRear = new WPI_TalonFX(Constants.LEFT_REAR_DRIVE);


    configureTalon();
    
  }


   // configure talon properties
   public static void configureTalon() {
    // JDE: Are current limits set - should they be set here or elsewhere?
    // https://docs.ctre-phoenix.com/en/latest/ch13_MC.html#new-api-in-2020
    rightFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
		rightFront.configNeutralDeadband(0.001, 0);
    rightFront.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    rightFront.configPeakOutputForward(PEAK_OUTPUT);
		rightFront.configPeakOutputReverse(-PEAK_OUTPUT);
    rightFront.configNominalOutputForward(0, 30);
		rightFront.configNominalOutputReverse(0, 30);

    leftFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
		leftFront.configNeutralDeadband(0.001, 0);
    leftFront.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    leftFront.setInverted(true);
    leftFront.setSensorPhase(true);
    leftFront.configPeakOutputForward(PEAK_OUTPUT);
		leftFront.configPeakOutputReverse(-PEAK_OUTPUT);
    leftFront.configNominalOutputForward(0, DEFAULT_TIMEOUT);
		leftFront.configNominalOutputReverse(0, DEFAULT_TIMEOUT);

    //rightRear.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    rightRear.follow(rightFront);
		rightRear.configNeutralDeadband(0.001, 0);
    rightRear.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    rightRear.follow(rightFront, FollowerType.AuxOutput1);
    rightRear.configPeakOutputForward(PEAK_OUTPUT);
		rightRear.configPeakOutputReverse(-PEAK_OUTPUT);
    rightRear.configNominalOutputForward(0, 30);
		rightRear.configNominalOutputReverse(0, 30);
    rightRear.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    rightRear.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    // leftRear.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    leftRear.follow(leftFront);
		leftRear.configNeutralDeadband(0.001, 0);
    leftRear.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    leftRear.follow(leftFront, FollowerType.AuxOutput1);
    leftRear.setInverted(true);    
		leftRear.configPeakOutputForward(PEAK_OUTPUT);
		leftRear.configPeakOutputReverse(-PEAK_OUTPUT);
		leftRear.configNominalOutputForward(0, DEFAULT_TIMEOUT);
		leftRear.configNominalOutputReverse(0, DEFAULT_TIMEOUT);
    leftRear.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    leftRear.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  public void tankPercent(double left, double right) {
    leftFront.set(ControlMode.PercentOutput, left * 0.75);
    rightFront.set(ControlMode.PercentOutput, right * 0.75);
  }

  

}
