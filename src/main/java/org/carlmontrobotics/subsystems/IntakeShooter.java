package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Shooter.*;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
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
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV,kA);
    private TimeOfFlight distanceSensor = new TimeOfFlight(dsPort1); //make sure id port is correct here
    private TimeOfFlight distanceSenor2 = new TimeOfFlight(dsPort2); // insert 
    private double dsDepth = 9.97;
    private double detectDistance = 13;
    public IntakeShooter() {
        pidControllerOutake.setP(kP);
        pidControllerOutake.setD(kD);
        outakeEncoder.setPositionConversionFactor(Math.PI/360);
    }
    public double getGamePieceDistance1() {
        return Units.metersToInches((distanceSensor.getRange() - dsDepth) /1000);
    }
    public double getGamePieceDistance2() {
        return Units.metersToInches((distanceSenor2.getRange() - dsDepth) /1000);
    }
    public boolean gameDistanceSees1st() {
        return getGamePieceDistance1() < detectDistance;
    }
    public boolean gameDistanceSees2nd() {
        return getGamePieceDistance2() < detectDistance;
    }
    public void senseGamePieceStop() {
        if (gameDistanceSees1st()){    
            pidControllerIntake.setReference((-1), CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(-1/60));
            if(gameDistanceSees2nd()) {
                pidControllerIntake.setReference(0, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(0));
            }
    }
}
    //TODO: this shit is complicated as fuck
    public double calculateDistanceSensorNotes() {
        double a = 4 * Math.pow(distanceBetweenSensors,2);
        return 1.2;
    }
    @Override
    public void periodic() {
        senseGamePieceStop();
    }
    public void setRPMOutake(double rpm) {
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(rpm));
    }
    public void setRPMintake() {
        pidControllerIntake.setReference(-6000, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(-6000));
    }
    public void shoot(double distance) {
        double rpm = calculateDistanceForRPM();
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(rpm));
        //To be implemented
    }
    public void stopOutake() {
        pidControllerOutake.setReference(0,CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(0));
    }
    public void stopIntake() {
        pidControllerIntake.setReference(0,CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(0));
    }


    public double calculateDistanceForRPM() {
        //returns specific rpm based off of the distance and angle it is in
        double angleInDegrees = outakeEncoder.getPosition();
        double distance = 20; //Add limelight return distance thingy here
        double surfaceVelocity = 17;
        return 17; //super cool mathy stuff after tests and distances
        //Based off of distance it
    }

}
