// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  public AddressableLED m_strip;
  public AddressableLEDBuffer m_Buffer;
  private Alliance alliance;
  public boolean rainbowBool = false;
  /** Creates a new LED. */
  public LED() {
    alliance = DriverStation.getAlliance();
    m_Buffer = new AddressableLEDBuffer(Constants.BUFFERSIZE);
    m_strip = new AddressableLED(Constants.PWMPORT);
    m_strip.setLength(m_Buffer.getLength());
    m_strip.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rainbow(){
    rainbowBool=!rainbowBool;
  }

  public void update(){
    SmartDashboard.putString("Rainbow", rainbowBool+"");
    if(rainbowBool){
      SmartDashboard.putString("hisw", "hi");
      setColor(255, 0, 0);
    }else{
      setColor(0, 255, 0);
    }
  }


  public void setColor(int r, int g, int b){

    for(int i=0;i<m_Buffer.getLength();i++){
      m_Buffer.setRGB(i,  r, g, b);
    }
    m_strip.setData(m_Buffer);
  }



}
