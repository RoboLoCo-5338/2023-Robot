// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final int RIGHTREAR_MOTOR = 1;
  public static final int RIGHTFRONT_MOTOR = 2;
  public static final int LEFTREAR_MOTOR = 3;
  public static final int LEFTFRONT_MOTOR = 4;
  // public static final int ELEVATOR_MOTOR = 7;
  public static final int CONE_TIPPER = 0;
  // public static final int EFFECTOR_MOTOR = 5;
  // public static final int ARM_MOTOR = 6;

  public static final int ABUTTON = 1; 
  public static final int BBUTTON = 2; 
  public static final int XBUTTON = 3; 
  public static final int YBUTTON = 4; 
  public static final int LBBUTTON = 5; 
  public static final int RBBUTTON = 6;
  public static final int BACKBUTTON = 7; 
  public static final int STARTBUTTON = 8; 
  public static final int LEFTSTICKBUTTON = 9; 
  public static final int RIGHTSTICKBUTTON = 10;

  // Height presets.
  public static final int Height = 3;
  public static final int height1 = 1;
  public static final int height2 = 2;
  public static final int height3 = 3;
  public static final int height4 = 4;
  public static final int height5 = 5;

  //speed_multi
  public static final double speed_multi = 0.1;
  public static final double kp = 0.01;
  public static final double min_command = 0.001;
}