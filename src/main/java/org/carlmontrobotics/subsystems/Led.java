package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Led.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(100);
    private final AddressableLED led = new AddressableLED(9);
    private final IntakeShooter intakeshooter;
    private int midpoint = (int) ledBuffer.getLength() / 2;
    private int color = 1;

    // rounds down
    // public static final double midpoint = Math.floor(ledLength/2);
    // public static final int ledLength = led.getLength();

    public Led(IntakeShooter intakeShooter) {
        this.intakeshooter = intakeShooter;
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        setLedColor(DEFAULT_COLOR_BLUE, 0, getLength());
        // SmartDashboard.putNumber("color", color);

    }

    public void setLedColor(Color8Bit color, int start, int end) {
        for (int i = start; i < end; i++)
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        led.setData(ledBuffer);
    }

    public int getLength() {
        return ledBuffer.getLength();
    }

    @Override
    public void periodic() {
        if(intakeshooter.intakeIsOverTemp()) {
            setLedColor(RED_NEO_550_MOTOR, 0, getLength());
            return;
        }
        // System.err.println("skdjfksd");
        /* 
        // testing for nicholas
        if (SmartDashboard.getNumber("color", color) == 1) {
            setLedColor(DETECT_NOTE_YELLOW, 0, getLength());
            // when intake TOF detects, but outtake TOF does not the bottom half of the LEDs
            // become orange

        } else if (SmartDashboard.getNumber("color", color) == 2) {
            setLedColor(HOLDING_GREEN, 0, getLength());
            // when both TOFs detect and the end efforcter is holding the note the LEDS turn
            // green

        } 
        else if (SmartDashboard.getNumber("color", color) == 3) {
            setLedColor(DEFAULT_COLOR_BLUE, 0, getLength());
        }
         */
        if (intakeshooter.intakeDetectsNote() && !intakeshooter.outakeDetectsNote())
        {
            setLedColor(DETECT_NOTE_YELLOW, 0,getLength());
         //when intake TOF detects, but outtake TOF does not the bottom half of the
       
        
        }else if (intakeshooter.intakeDetectsNote() &&intakeshooter.outakeDetectsNote()) {
            setLedColor(HOLDING_GREEN, 0, getLength());
         //when both TOFs detect and the end efforcter is holding the note the LEDS turn green
       
        }else{
            setLedColor(DEFAULT_COLOR_BLUE, 0, getLength());
         //otherwise LEds are blue
        }
        

    }
    // nothing : setLedColor(defaultColr, 0, ledBuffer.getLength())
    // first tof detect: setLedColor(detectNote, 0, midpoint)
    // second tof detec: setLedColor(detectNote, midpoint, ledBuffer.getLength())
    // both tof detect: setLedColor(holding, 0, ledBuffer.getLength())

}