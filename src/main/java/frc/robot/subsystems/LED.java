package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class LED extends SubsystemBase  {
    private final AddressableLED m_led = new AddressableLED(Constants.PWMPORT);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.BUFFERSIZE);
    private int m_rainbowFirstPixelHue;
 
    public LED(){
        m_led.setLength(m_ledBuffer.getLength());
        setLED(0 ,0 ,255);
        m_led.setData(m_ledBuffer);
        m_led.start();
  m_rainbowFirstPixelHue=1;
    }
 
 
 public void periodic(){




    //if(Constants.ALLIANCE.equalsIgnoreCase("Blue")){
         setLED(0 ,0 ,255);
         m_led.setData(m_ledBuffer);
         m_led.start();
    // }else if(Constants.ALLIANCE.equalsIgnoreCase("Red")){
    //     setLED(255,0,0);
    // }


 }




    public void setLED(int r, int g, int b){




        for(int i=0;i<m_ledBuffer.getLength();i++){
            m_ledBuffer.setRGB(i, r, g, b);
}
}


public void rainbow(){
    for( int i = 0; i < m_ledBuffer.getLength(); i++){
        int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
   
    m_ledBuffer.setHSV(i, hue, 255,128);
}
m_rainbowFirstPixelHue+=3;
m_rainbowFirstPixelHue%=180;
m_led.setData(m_ledBuffer);




}




public void preset(int preset){
	if(preset==0){
		setLED(0, 153, 153);
	}else if(preset==1){
		setLED(102, 255, 255);
	}else if(preset==2){
	setLED(255,153,255);
}else{
setLED(255,50,150);


}
}


















 }












 
 
 
 







