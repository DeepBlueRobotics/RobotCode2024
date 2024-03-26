package org.carlmontrobotics.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.carlmontrobotics.subsystems.IntakeShooter;

import com.ctre.phoenix.led.ColorFlowAnimation;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static org.carlmontrobotics.Constants.Effectorc.*;
import static org.carlmontrobotics.Constants.Led.ledLength;
import static org.carlmontrobotics.Constants.Led.startingColor;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.commands.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * AuxSystems contains both LED and Rumble functionality,
 * and is separate because of it's special needs to take in other subsystems.
*/
public class AuxSystems extends SubsystemBase {
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.Led.ledLength);
    private final AddressableLED led = new AddressableLED(Constants.Led.ledPort);
    private int midpoint = (int) Math.floor(ledBuffer.getLength()/2);//rounds down
    //AuxSubsystems watches the two other subsystems in order to rumble/led in peace
    //public final Color8Bit defaultColor = new Color8Bit(255,0,0);
    private final Arm arm;
    private final IntakeShooter effector;
    //for led color ramping
    private Color8Bit currentColor = new Color8Bit(0,0,0);


    private LedPoint currentPoint = new LedPoint();
    public boolean matchMode = true;
    //^ whether we use normal color checking or flowing color mode

    private double rumble;
    //stuf
    private Color8Bit[] colorCycle = {
        new Color8Bit(255,0,0),//0 red
        new Color8Bit(255,127,0),//1 orange
        new Color8Bit(255,255,0),//2 yellow
        new Color8Bit(127,255,0),
        new Color8Bit(0,255,0),//4 blue
        new Color8Bit(0,255,127),
        new Color8Bit(0,255,255),
        new Color8Bit(0,127,255),
        new Color8Bit(0,127,255),
        new Color8Bit(0,0,255),//9 green
        new Color8Bit(127,0,255),
        new Color8Bit(255,0,255),//11 purple
        new Color8Bit(255,0,127),
        new Color8Bit(0,0,0)//13 black
    };

    public AuxSystems(Arm a, IntakeShooter is){
        this.arm = a;
        this.effector = is;
        //leds
        led.setLength(ledBuffer.getLength());
        setLedColor(startingColor, 0 , ledBuffer.getLength());
        led.start();


        SmartDashboard.putBoolean("LED Match Mode:", matchMode);

        // int r = 0;
        // int g = 0;
        // int b = 0;
        // //weird alex stuff
        // for (int red = 0; red < 255/15; red ++){
        //     for (int blue = 0; blue < 255/15; blue ++){
        //         for (int green = 0; green < 255/15; green ++){
        //             g +=15;

        //             currentPoint = currentPoint.addnext(0.1,new Color8Bit(r,g,b));

        //         }
        //         g = 0;
        //         b +=15;


        //     }
        //     b = 0;
        //     r +=15;


        //}
        for (Color8Bit color : colorCycle){
            currentPoint = currentPoint.addnext(2,color);

        }



        currentPoint.start();
    }

    public void setLedColor(Color8Bit color, int start, int end) {
        for (int i = start; i < end ; i++)
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
            led.setData(ledBuffer);
    }
    @Override
    public void periodic(){
        matchMode = SmartDashboard.getBoolean("LED Match Mode:", matchMode);
        if (matchMode){
            /** LED RULES
             * only intake TOF: lower yellow
             * only outake TOF: upper purple
             * both TOF:        all green
             * none:            all red
            */
            /** RUMBLE RULES
             * at setpoint + intake TOF: rumble
             * only outake TOF:          rumble
             * else: no rumble
             * none:            all red
            */
            if (effector.intakeDetectsNote() && effector.outakeDetectsNote()){
                setLedColor(colorCycle[0], 0, ledLength);
            }else if (effector.intakeDetectsNote()){
                setLedColor(colorCycle[13], 0, ledLength);//all black
                setLedColor(colorCycle[2], 0, midpoint);
            } else if (effector.outakeDetectsNote()) {
                setLedColor(colorCycle[13], 0, ledLength);//all black
                setLedColor(colorCycle[11], midpoint, ledLength);
            } else {
                setLedColor(colorCycle[0], 0, ledLength);
            }


        }else {
            currentColor = currentPoint.currentDesiredColor();
            setLedColor(currentColor,0,Constants.Led.ledLength);


        }



    }

    /**
    * LedPoint
    *
    * Used to represent points of a graph of color, so that you can smoothly transition between them.
    * Create an initial point by calling
    *     LedPoint ledpoint = new LedPoint();
    * or so, and then add points to the graph with
    *     ledpoint = ledpoint.addnext(2, new Color8Bit(255,0,0));
    * which would smoothly transition from black (000000) to full red (ff0000) in two seconds
    * as long as you called
    *     setLedColor(ledpoint.currentDesiredColor() ,0,Constants.Led.ledLength);
    * every periodic
    */
    public class LedPoint {
        public double startTime,duration;
        public int r,g,b;
        public LedPoint parent,child;

        //initial black null constructor
        public LedPoint() {
            this.parent=null;
            this.startTime = Timer.getFPGATimestamp();
            this.duration = 0;
            this.setColor(new Color8Bit(0,0,0));
        }
        //use parent to chain points together
        public LedPoint(LedPoint parent, double timeToTarget, Color8Bit color) {
            this.parent=parent;
            this.duration = timeToTarget;
            this.setColor(color);
        }
        public double start(){
            double now = Timer.getFPGATimestamp();

            if (parent!=null){
                startTime = parent.start() + parent.duration;
            } else {
                startTime = now;
            }
            return startTime;
        }
        //default assumes startTime is NOW
        public LedPoint addnext(double timeToTarget, Color8Bit color) {
            this.child = new LedPoint(this, timeToTarget, color);
            return this.child;
        }

        private void setColor(Color8Bit color){
            this.r=color.red;
            this.g=color.green;
            this.b=color.blue;
        }

        public Color8Bit currentDesiredColor(){//return the color it should be now
            double currTime = Timer.getFPGATimestamp();
            if (currTime<startTime){//is it our parents turn still?
                if (parent==null){//no parent, do nothing
                    return currentColor;
                } else{
                    return parent.currentDesiredColor();
                }
            }
            double endTime = this.startTime+this.duration;

            double dfrac = (endTime-currTime)/(endTime-startTime);
            //^ fraction of completion
            return new Color8Bit(
                currentColor.red   + (int) (dfrac*(r-currentColor.red)),
                currentColor.green + (int) (dfrac*(g-currentColor.green)),
                currentColor.blue  + (int) (dfrac*(b-currentColor.blue))
            );
        }
    }
}
