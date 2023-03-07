package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConeTipper extends SubsystemBase{
    private CANSparkMax coneTipperMotor;
    private RelativeEncoder coneTipperEncoder;
    private SparkMaxPIDController coneTipperPidController;
    private double[] locations = {0.0,0.1};
    private int location=0;

    public static double coneTipperP=0.1;
    public static double coneTipperI=0.0;
    public static double coneTipperD=0.0;
    public static double coneTipperFF=0.0;

    public static double coneChange=0.0;

    public ConeTipper(){
        coneTipperMotor = new CANSparkMax(Constants.CONE_TIPPER, MotorType.kBrushless);
        coneTipperEncoder = coneTipperMotor.getEncoder();
        coneTipperPidController = coneTipperMotor.getPIDController();

        configController();
    }


    private void configController(){

        coneTipperPidController.setP(coneTipperP);
        coneTipperPidController.setI(coneTipperI);
        coneTipperPidController.setD(coneTipperD);
        coneTipperPidController.setFF(coneTipperFF);

    }

    public void setConeChange( Elevator elevator){
        location++;
        //TODO change values to actual values
        if(!(location%2==0&&elevator.getElevatorPosition()<0)){
            double current = coneTipperEncoder.getPosition();
            SmartDashboard.putNumber("Elevator Position", location%2);
            coneChange = locations[location%2] - current;
        }
      
    }

    public void stopConeTipper(){
        coneTipperMotor.set(0);
    }

    public void setPosition(){
        coneTipperPidController.setReference(coneChange, ControlType.kPosition);
    }

    public void moveConeTipper(double speed){
        coneTipperMotor.set(speed);
    }

    public void resetConeTipper(){
        coneTipperEncoder.setPosition(0);
    }

    public double getConeTipperPosition(){
        return coneTipperEncoder.getPosition();
    }

}
