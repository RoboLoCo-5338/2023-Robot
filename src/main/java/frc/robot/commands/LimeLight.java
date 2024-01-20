// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

// @SuppressWarnings("Serial Warnings")
// public class LimeLight extends CommandBase {

//   private NetworkTable table;
//   private double x;//tx value
//   private double y;//ty value
//   private double area;//ta value
//   private double p;//multiplier value (proportional speed for turning depending on tx value)
//   public double pipeLineNum=0;
//   /** Creates a new LimeLight + sets initial values. */
  
//   public LimeLight() {
//     table = NetworkTableInstance.getDefault().getTable("limelight");
//     x=0;
//     y=0;
//     area=0;
//     p=0.02;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     SmartDashboard.putString("HELLO", "HELLO");  
//     NetworkTableEntry tx = table.getEntry("tx");
//     NetworkTableEntry ty = table.getEntry("ty");
//     NetworkTableEntry ta = table.getEntry("ta");

//     //read values periodically + get 
//     x = tx.getDouble(0.0);
//     y  = ty.getDouble(0.0);
//     area = ta.getDouble(0.0);

//     //post to smart dashboard periodically
//     SmartDashboard.putNumber("LimelightX", x);  
//     SmartDashboard.putNumber("LimelightY", y);
//     SmartDashboard.putNumber("LimelightArea", area);
//     //System.out.println(x);
  
//     //sets velocity value for turning towards the target
//     double left= -p*x > 0 ? Math.min(-p*x, 0.3) : Math.max(-p*x, -0.3);//basically an if else statement; if it's greater, choose the smaller of the values, else chooses the max of the other two values
//     double right= p*x > 0 ? Math.min(p*x, 0.3) : Math.max(p*x, -0.3);

//     if(x!=0){//turns when not facing the april tag/when the tag is not in sight
//       RobotContainer.drivetrain.tankDrive(left, right);
//     }
//   }

//   public double getX(){//getter for x
//     return x;
//   }

//   public double getY(){//getter for y
//     return y;
//   }

//   public void setPipeline() {
// 		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
//         pipeLineNum=(pipeLineNum+1)%2;
//     	pipelineEntry.setNumber(pipeLineNum);
//     }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
