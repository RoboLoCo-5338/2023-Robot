// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends CommandBase {

  private NetworkTable table;
  private double x;//tx value
  private double y;//ty value
  private double area;//ta value
  private double p;//multiplier value (proportional speed for turning depending on tx value)
  private double[] botpose;
  private double[] campose;

  /** Creates a new LimeLight + sets initial values. */
  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    x=0;
    y=0;
    area=0;
    p=0.02;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("HELLO", "HELLO");  
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tBotpose = table.getEntry("botpose");
    NetworkTableEntry tCampose = table.getEntry("campose");

    //read values periodically + get 
    x = tx.getDouble(0.0);
    y  = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    botpose = tBotpose.getDoubleArray(new double[6]);
    campose = tCampose.getDoubleArray(new double[6]);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);  
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumberArray("botpose", botpose);
    SmartDashboard.putNumberArray("campose", campose);
    //System.out.println(x);
  
    //sets velocity value for turning towards the target
    double left= -p*x > 0 ? Math.min(-p*x, 0.3) : Math.max(-p*x, -0.3);//basically an if else statement; if it's greater, choose the smaller of the values, else chooses the max of the other two values
    double right= p*x > 0 ? Math.min(p*x, 0.3) : Math.max(p*x, -0.3);

    // Drive towards target by aligning with crosshair
    // TODO Calibrate cross hair to light up with top of target (https://docs.limelightvision.io/en/latest/cs_autorange.html)
    double distance = y * 0.1;
    left += distance;
    right += distance;

    if(x!=0){//turns when not facing the april tag/when the tag is not in sight
      RobotContainer.driveSystem.tankDriveVelocity(left, right);
    }
  }

  public double getX(){//getter for x
    return x;
  }

  public double getY(){//getter for y
    return y;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
