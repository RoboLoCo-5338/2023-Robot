package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private CANSparkMax armMotor;
    private RelativeEncoder armEncoder;
    private SparkMaxPIDController armController;
    public double[] armHeights = {27,100,-4,35,0,90}; //presets 2-4 for stow/unstow, DOUBLE CHECK 0 & 1
    public static double armChange = 0;

    public static double armP=0.1;
    public static double armI=0.0;
    public static double armD=0.0;
    public static double armFeed_Forward=0.0;

    public Arm(){
        
        armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);
        armEncoder = armMotor.getEncoder();
        armController = armMotor.getPIDController();
        armController.setOutputRange(-0.4, 0.4);
        //armEncoder.setPositionConversionFactor(1);
        armMotor.setSmartCurrentLimit(40);
  
        configController();
    }

    private void configController(){
    
     
    
        // PID config for the arm
        armController.setP(armP);
        armController.setI(armI);
        armController.setD(armD);
        armController.setFF(armFeed_Forward);
      }

    
      public void resetArm(){
        armEncoder.setPosition(0);
      }
  
      public double getArmPosition(){
        return armEncoder.getPosition();
      }
  
      public void stopArm(){  
        armMotor.set(0);
      }

      public void setArm(int preset){
        SmartDashboard.putString("unstow", "unstow");
        armController.setReference(armHeights[preset], CANSparkMax.ControlType.kPosition);
      }
    
      public void moveArm(double speed){
        if(getArmPosition() > -20 && speed < 0) { //CHANGE MAYBE
          SmartDashboard.putNumber("Arm Position Teleop", getArmPosition());
          armMotor.set(speed);
        }
        if(getArmPosition() < 100 && speed > 0) { //CHANGE MAYBE
          SmartDashboard.putNumber("Arm Position Teleop", getArmPosition());
          armMotor.set(speed);
        }
      }
      
      public void setArmChange(int preset){
        double current = armEncoder.getPosition();
        SmartDashboard.putNumber("Arm Preset", preset);
        SmartDashboard.putNumber("Arm position arm change", armEncoder.getPosition());
        armChange = armHeights[preset] - current;
        SmartDashboard.putNumber("Arm change", armChange);
      }
  
    
}
