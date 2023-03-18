// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double ksVolts = 0.18583;
  public static final double kvVoltSecondsPerMeter = 2.3892;
  public static final double kaVoltSecondsSquaredPerMeter = 0.48446;
  public static final double kPDriveVel = 3;

  //CHANGE THIS VALUE LATER
  public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
  
  public static final double kMaxSpeedMetersPerSecond = 0.25;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

      //CAN IDs
      public static final int RIGHT_FRONT_DRIVE = 2;
      public static final int RIGHT_REAR_DRIVE = 3;
      public static final int LEFT_FRONT_DRIVE = 0;
      public static final int LEFT_REAR_DRIVE = 1;


}
