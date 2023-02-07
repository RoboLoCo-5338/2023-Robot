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
  private double x;
  private double y;
  private double area;
  private double p;


  /** Creates a new LimeLight. */
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

//read values periodically
x = tx.getDouble(0.0);
y = ty.getDouble(0.0);
area = ta.getDouble(0.0);

  //post to smart dashboard periodically
  SmartDashboard.putNumber("LimelightX", x);  
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightArea", area);
  System.out.println(x);
  
  double left= -p*x > 0 ? Math.min(-p*x, 0.3) : Math.max(-p*x, -0.3);
  double right= p*x > 0 ? Math.min(p*x, 0.3) : Math.max(p*x, -0.3);

  if(x!=0){
    RobotContainer.driveSystem.tankDriveVelocity(left, right);
  }
  


  }

  public double getX(){
    return x;
  }

  public double getY(){
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
