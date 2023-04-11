package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    private boolean reverse = false;
    private final AddressableLED m_led = new AddressableLED(Constants.PWMPORT);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.BUFFERSIZE);
    private int m_rainbowFirstPixelHue;
    private int m_teamColorsFirstPixelHue;
    private boolean reverseBool;
    private boolean rainbowBool;
    private boolean teamColorsBool;

    public LED {
        reverse
        m_led.setLength(m_ledBuffer.getLength());
        setLED(0, 0, 255);
        m_led.setData(m_ledBuffer);
        m_led.start();
        m_rainbowFirstPixelHue = 1;
        m_teamColorsFirstPixelHue = 60;

    }

    public void update() {

        if (reverseBool){
            
            setColor(253,218,13);

        }else if(rainbowBool){
           for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            int hue = (m_teamColorFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

            m_ledBuffer.setHSV(i, hue, 255, 128);
            }

            m_teamColorsFirstPixelHue += 3;
            m_teamColorsFirstPixelHue %= 180;

        }else if(teamColorBool){
            //60 to 250
             for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            int hue = (m_teamColorFirstPixelHue + (i * 190 / m_ledBuffer.getLength())) % 190;

            m_ledBuffer.setHSV(i, hue, 255, 128);
            }

            m_teamColorsFirstPixelHue += 3;
            m_teamColorsFirstPixelHue = m_teamColorsFirstPixelHue > 250 ? 60 : m_teamColorFirstPixelHue;
        }else {
           alliance();
        }

        m_led.setData(m_ledBuffer);
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
    
    public void reverse(){ reverseBool = !reverseBool; }
    
    

    public void setColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }


    public void rainbow() { rainbowBool = !rainbowBool; }

     public void teamColors() { teamColorsBool = !teamColorsBool; }


   
}