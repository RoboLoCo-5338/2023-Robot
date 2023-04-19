package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final AddressableLED m_led = new AddressableLED(Constants.PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.BUFFERSIZE);
  private int m_rainbowFirstPixelHue;
  private int m_teamFirstPixelHue;

  private int reverse=-1;
  private int rainbow=-1;

  public LED() {
      m_led.setLength(m_ledBuffer.getLength());
      setLED(0, 0, 255);
      m_led.setData(m_ledBuffer);
      m_led.start();
      m_rainbowFirstPixelHue = 1;
      setLED(0, 0, 255);

      m_teamFirstPixelHue = 60;
  }

  @Override 
  public void periodic() {
    
    }

    public void update(){
        if (reverse>0) {
            setLED(255, 255, 0);
            // green leds when the robot is in reverse
        } else {
            alliance();
        }
    
        m_led.setData(m_ledBuffer);
        m_led.start();
    
    }

    public void updateRainbow(){
        if (reverse>0) {
            setLED(255, 255, 0);
            // green leds when the robot is in reverse
        } else {
            alliance();
        }
    
        m_led.setData(m_ledBuffer);
        m_led.start();
    
    }
    public void reverse(){
        reverse *= -1;
    }

  
    

    public void alliance() {
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            setLED(0, 0, 255);
            // blue alliance lights
        } else if (DriverStation.getAlliance().equals(Alliance.Red)) {
            setLED(255, 0, 0);
            // red alliange lights
        }
    }



    public void setLED(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
            m_ledBuffer.setRGB(i, r, g, b);

        }
    }

    public void rainbow() {
       // rainbow*=-1;
        //if(rainbow>0){
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
   // }else{
        //setLED(0, 0, 0);
   // }
        m_led.setData(m_ledBuffer);

    }

    public void teamColors(){
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            int hue = (m_teamFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 120;

            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_teamFirstPixelHue += 3;
        m_teamFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }
}
