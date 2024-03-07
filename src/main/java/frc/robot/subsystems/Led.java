package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class Led {
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.Led.ledLength);
    private final AddressableLED led = new AddressableLED(Constants.Led.ledPort);


    double bottomHalf = Math.floor(ledBuffer.getLength()/2); //rounds down
    double topHalf = ledBuffer.getLength() - bottomHalf;
    double all = ledBuffer.getLength();

    int index = 1;
    public Led(){
        led.setLength(ledBuffer.getLength());
        setLedColor(Constants.Led.defaultColor);
        led.start();
    }
    public void setLedColor(Color8Bit color) {
        for (int i = 0; i < ledBuffer.getLength() ; i++)
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        led.setData(ledBuffer);
    }
     
        /*in arm code make it so that color changes to intakeColor when distance sensors detect a note
        using either boolean noteInIntake or boolean intakeDetectsNote
        
        public void changeIntakeColor(){
            setLedColor(intakeColor);
            resetColorCommand.schedule(); 
        }

        if(noteInIntake){
            changeIntakeColor();   
        }
        

       private Command resetColorCommand = new SequentialCommandGroup(
            new WaitCommand(ledDefaultColorRestoreTime),
            new InstantCommand(() -> setLedColor(defaultColor))) {
            public boolean runsWhenDisabled() {
            return true;
         };
        }; */
}


