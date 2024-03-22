package org.carlmontrobotics.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.carlmontrobotics.Constants.Led.*;


public class Led extends SubsystemBase{
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(getLength());
    private final AddressableLED led = new AddressableLED(ledPort);
    //private double midpoint = Math.floor(ledBuffer.getLength()/2); 
    //rounds down
    // public static final double midpoint = Math.floor(ledLength/2); 
    //public static final int ledLength = led.getLength();

    
    public Led(){
        led.setLength(ledBuffer.getLength());
        setLedColor(DEFAULT_COLOR, 0 , ledBuffer.getLength());
        led.start();
    }
    public void setLedColor(Color8Bit color, int start, int end) {
        for (int i = start; i < end ; i++)
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        led.setData(ledBuffer);
    }
    public int getLength(){
        return ledBuffer.getLength();
    }
    // nothing : setLedColor(defaultColr, 0, ledBuffer.getLength())
    //first tof detect: setLedColor(detectNote, 0, midpoint)
    //second tof detec: setLedColor(detectNote, midpoint, ledBuffer.getLength())
    //both tof detect: setLedColor(holding, 0, ledBuffer.getLength())
   
}