package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;//need to make a constant for the LED

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(LED.PWMPORT);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.BUFFERSIZE);
    private int m_rainbowFirstPixelHue;
    private Elevator elevator;
    
    public LEDSubsystem(Elevator elevator){
    this.elevator = ev;
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    }

    @Override
    public void periodic(){
        setLEDs();
    }

    private void roboLocoColor(){
      setAll(Color.kYellow); 
    }

    private void setLEDs() {
      m_led.setData(m_ledBuffer);
    }

    private void setAll(Color color) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, color);
      }
    }
}


  
  

  

   
  
   
    
  
    
  

