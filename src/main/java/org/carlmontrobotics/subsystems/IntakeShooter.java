package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Shooter.*;
import static org.mockito.ArgumentMatchers.matches;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.subsystems.Arm;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO_550);
    private final CANSparkMax outakeMotor = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO_550);
    private final RelativeEncoder outakeEncoder = outakeMotor.getEncoder();
    private final SparkPIDController pidControllerOutake = outakeMotor.getPIDController();
    private final SparkPIDController pidControllerIntake = intakeMotor.getPIDController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private TimeOfFlight distanceSensor = new TimeOfFlight(dsPort1); // make sure id port is correct here
    private TimeOfFlight distanceSenor2 = new TimeOfFlight(dsPort2); // insert
    private double dsDepth = 9.97;
    private double detectDistance = 13;

    public IntakeShooter() {
        pidControllerOutake.setP(kP);
        pidControllerOutake.setD(kD);
        outakeEncoder.setPositionConversionFactor(Math.PI / 360);
    }

    public double getGamePieceDistance1() {
        return Units.metersToInches((distanceSensor.getRange() - dsDepth) / 1000);
    }

    public double getGamePieceDistance2() {
        return Units.metersToInches((distanceSenor2.getRange() - dsDepth) / 1000);
    }

    public boolean gameDistanceSees1st() {
        return getGamePieceDistance1() < detectDistance;
    }

    public boolean gameDistanceSees2nd() {
        return getGamePieceDistance2() < detectDistance;
    }

    public void senseGamePieceStop() {
        if (gameDistanceSees1st()) {
            pidControllerIntake.setReference((-1), CANSparkBase.ControlType.kVelocity, 0,
                    feedforward.calculate(-1 / 60));
            if (gameDistanceSees2nd()) {
                pidControllerIntake.setReference(0, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(0));
            }
        }
    }
    //Find offset of note from the center line using big mathy mathy, god I hope this works chatgpt gave me the formulas :))))))
    public double calculateDistanceSensorNotes() {
        double center = 11.485;
        double d1 = getGamePieceDistance1();
        double d2 = getGamePieceDistance2();
        double r = 7;
        //double xm = (0 + distanceBetweenSensors)/2; //X midpoint between 2 points
        double ym = (d1+d2)/2; //Y midpoint between 2 points
        double m = (0-distanceBetweenSensors)/(d2-d1); // Slope
        //double h = xm + r * (1/(Math.sqrt(1+Math.pow(m, 2)))); // x cord of center <- currently incorrect, check this link for correct:https://stackoverflow.com/questions/36211171/finding-center-of-a-circle-given-two-points-and-radius 
        double k = ym + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2, 2)) * (distanceBetweenSensors))/r;// y cord of center
        //Take into note that in reality, the 2 points can return 2 possible centers
        return k - center; //<- offset from the center
        // double a = 4 * Math.pow(distanceBetweenSensors,2); <- for speaker calcs
        // return 1.2;
    }

    public double calculateIntakeAmount(){
        //Literatly just calcDIstanceSensorNotes but instead of solving for k, we are solving for h
        double d1 = getGamePieceDistance1();
        double d2 = getGamePieceDistance2();
        double r = 7;

        double xm = (distanceBetweenSensors)/2;

        double h = xm + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2,2)) * (d1-d2))/r; 
        return h;
    }

    @Override
    public void periodic() {
        senseGamePieceStop();
    }

    public void setRPMOutake(double rpm) {
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(rpm));
    }

    public void setRPMintake() {
        pidControllerIntake.setReference(-6000, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(-6000));
    }

    public void shoot(double distance) {
        double rpm = calculateDistanceForRPM();
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(rpm));
        // To be implemented
    }

    public void stopOutake() {
        pidControllerOutake.setReference(0, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(0));
    }

    public void stopIntake() {
        pidControllerIntake.setReference(0, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(0));
    }

    public double calculateDistanceForRPM() {
        // returns specific angle + rpm given distance from speaker
        double distance = 20; // Add limelight return distance thingy here
        double aprilTagToSpeaker = 21;
        double hypotenuse = Math.sqrt(Math.pow(distance,2) + Math.pow(aprilTagToSpeaker, 2));
        return 17; // super cool mathy stuff after tests and distances
        // Based off of distance it
    }
    
    public double calculateOffsetAndAngle(){
        //returns optimal offest and shooting angle given the distance from the april tag
        double distance = 20; //Karena, do cool limelight stuff here or something
        double currentAngle; // = 180 + (armEncoder.getPostion()*360) - 115
        double currentOffset; // Gotta find the equation for this, should depend on angle, just need my calculator to work and not give me an r^2 of .4
        //GEt data from outtake testing to find the function of distance to angle to find the optimal angle and offset for the endeffector for the best shot
        
        return 15; //Cool stuff after calcs
    }
}
