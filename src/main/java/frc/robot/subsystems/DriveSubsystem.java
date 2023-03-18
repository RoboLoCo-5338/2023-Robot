package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.print.attribute.standard.MediaSize.NA;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

  private static WPI_TalonFX rightFront;
  private static WPI_TalonFX rightRear;
  private static WPI_TalonFX leftFront;
  private static WPI_TalonFX leftRear;
  private static double PEAK_OUTPUT = 0.2;
  private static final int DEFAULT_TIMEOUT = 30;
  private static final double GEAR_RATIO = 10.7 / 1.0;

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors ;

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors; 

  // The robot's drive
  private final DifferentialDrive m_drive;


  // The gyro sensor
  private AHRS Navx;
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT_DRIVE);
    rightRear = new WPI_TalonFX(Constants.RIGHT_REAR_DRIVE);
    leftFront = new WPI_TalonFX(Constants.LEFT_FRONT_DRIVE);
    leftRear = new WPI_TalonFX(Constants.LEFT_REAR_DRIVE);
    Navx= new AHRS(SPI.Port.kMXP);
    configureTalon();
    
    m_leftMotors= new MotorControllerGroup(leftRear,leftFront);

    m_rightMotors=
    new MotorControllerGroup(
        rightRear,
        rightFront);

        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors); 

    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(Navx.getAngle()),leftFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI, rightFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI);
  }

  
  // configure talon properties
  public static void configureTalon() {
    // JDE: Are current limits set - should they be set here or elsewhere?
    // https://docs.ctre-phoenix.com/en/latest/ch13_MC.html#new-api-in-2020
    rightFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
		rightFront.configNeutralDeadband(0.001, 0);
    rightFront.configClosedLoopPeakOutput(0, PEAK_OUTPUT, 100);
    //rightFront.setInverted(true);
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
    //rightRear.setInverted(true);
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
    leftRear.setInverted(true);
    leftRear.setSensorPhase(true);
    leftRear.follow(leftFront, FollowerType.AuxOutput1);    
		leftRear.configPeakOutputForward(PEAK_OUTPUT);
		leftRear.configPeakOutputReverse(-PEAK_OUTPUT);
		leftRear.configNominalOutputForward(0, DEFAULT_TIMEOUT);
		leftRear.configNominalOutputReverse(0, DEFAULT_TIMEOUT);
    leftRear.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    leftRear.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      new Rotation2d(Navx.getAngle()), leftFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI, rightFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftFront.getSelectedSensorVelocity() / GEAR_RATIO / 2048*0.1524, rightFront.getSelectedSensorVelocity() / GEAR_RATIO / 2048*0.1524);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
  //   m_odometry.resetPosition(
  //     new Rotation2d(Navx.getAngle()),leftFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI,rightFront.getSelectedSensorPosition()/2048/GEAR_RATIO*0.1524*Math.PI, pose);
  // }

  m_odometry.resetPosition(
    new Rotation2d(Navx.getAngle()),0,0, pose);
}

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    leftRear.setSelectedSensorPosition(0);
    rightRear.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (rightFront.getSelectedSensorPosition()*0.1524*Math.PI + leftFront.getSelectedSensorPosition()*0.1524*Math.PI) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public TalonFXSensorCollection getLeftEncoder() {
    return leftFront.getSensorCollection();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public TalonFXSensorCollection getRightEncoder() {
    return rightFront.getSensorCollection();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    Navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return new Rotation2d(Navx.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -Navx.getRate();
  }

  public void tankPercent(double left, double right) {
    leftFront.set(ControlMode.PercentOutput, left );
    rightFront.set(ControlMode.PercentOutput, right );
  }
}