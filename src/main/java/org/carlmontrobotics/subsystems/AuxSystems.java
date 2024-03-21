package org.carlmontrobotics.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.carlmontrobotics.subsystems.IntakeShooter;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.commands.Intake;


/** 
 * AuxSystems contains both LED and Rumble functionality, 
 * and is separate because of it's special needs to take in other subsystems.
*/
public class AuxSystems {
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.Led.ledLength);
    private final AddressableLED led = new AddressableLED(Constants.Led.ledPort);
    private int midpoint = (int) Math.floor(ledBuffer.getLength()/2);//rounds down
    
    private final IntakeShooter endEffector;

    
    public AuxSystems(IntakeShooter is){
        endEffector = is;
        //leds
        led.setLength(ledBuffer.getLength());
        setLedColor(Constants.Led.defaultColor, 0 , ledBuffer.getLength());
        led.start();
    }

    public void setLedColor(Color8Bit color, int start, int end) {
        for (int i = start; i < end ; i++)
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
            led.setData(ledBuffer);
    }
}

 // nothing : setLedColor(defaultColr, 0, ledBuffer.getLength())
    //first tof detect: setLedColor(detectNote, 0, midpoint)
    //second tof detec: setLedColor(detectNote, midpoint, ledBuffer.getLength())
    //both tof detect: setLedColor(holding, 0, ledBuffer.getLength())
     
        /*in arm code make it so that color changes to intakeColor when distance sensors detect a note
        (using either boolean noteInIntake or boolean intakeDetectsNote)
        there is definitely a shorter way to write this
        
        public void changeIntakeColor(){
            setLedColor(intakeColor);
            resetColorCommand.schedule(); 
        }

        if(noteInIntake){
            changeIntakeColor();   
        }

        public void changeOuttakeColor(){
            setLedColor(outtakeColor);
            resetColorCommand.schedule(); 
        }

        if(/boolean value for outtake detection/){
            changeOuttakeColor();   
        }
        
        public void changeIntakeOuttakeColor(){
            setLedColor(intakeouttakeColor);
            resetColorCommand.schedule(); 
        }

        if(noteInIntake && /boolean value for outtake detection/){
            changeIntakeOuttakeColor();   
        }

       private Command resetColorCommand = new SequentialCommandGroup(
            new WaitCommand(ledDefaultColorRestoreTime),
            new InstantCommand(() -> setLedColor(defaultColor))) {
            public boolean runsWhenDisabled() {
            return true;
         };
        }; */


