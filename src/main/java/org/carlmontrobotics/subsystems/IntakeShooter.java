package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.IntakeShoot;

import static org.carlmontrobotics.Constants.IntakeShoot.*;
import static org.mockito.ArgumentMatchers.matches;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import org.carlmontrobotics.subsystems.Limelight;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(INTAKE_PORT, MotorConfig.NEO_550);
    private final CANSparkMax outakeMotor = MotorControllerFactory.createSparkMax(OUTAKE_PORT, MotorConfig.NEO_550);
    private final RelativeEncoder outakeEncoder = outakeMotor.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final SparkPIDController pidControllerOutake = outakeMotor.getPIDController();
    private final SparkPIDController pidControllerIntake = intakeMotor.getPIDController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);//for both intake and outtake 
    private TimeOfFlight intakeDistanceSensor = new TimeOfFlight(INTAKE_DISTANCE_SENSOR_PORT); // make sure id port is correct here
    private TimeOfFlight OutakeDistanceSensor = new TimeOfFlight(OUTAKE_DISTANCE_SENSOR_PORT); // insert\
    private final Limelight limelight = new Limelight();
    
    
    public IntakeShooter() {
        //Figure out which ones to set inverted
        intakeMotor.setInverted(INTAKE_MOTOR_INVERSION);
        outakeMotor.setInverted(OUTAKE_MOTOR_INVERSION);         
        pidControllerOutake.setP(kP[0]);
        pidControllerOutake.setI(kI[0]);
        pidControllerOutake.setD(kD[0]);
        pidControllerIntake.setP(kP[1]);
        pidControllerIntake.setI(kI[1]);
        pidControllerIntake.setD(kD[1]);
    }
    //---------------------------------------------------------------------------------------------------
    //checking whether RPM is within tolerance
    public boolean isWithinTolerance(double outakeRPM){
        return outakeEncoder.getVelocity()<outakeRPM+RPM_TOLERANCE && outakeRPM-RPM_TOLERANCE<outakeEncoder.getVelocity();
    }
    //---------------------------------------------------------------------------------------------------
    private double getGamePieceDistanceIntake() {
        return Units.metersToInches((intakeDistanceSensor.getRange() - DS_DEPTH) / 1000);
    }

    private double getGamePieceDistanceOutake() {
        return Units.metersToInches((OutakeDistanceSensor.getRange() - DS_DEPTH) / 1000);
    }

    private boolean intakeDetectsNote() {
        return getGamePieceDistanceIntake() < DETECT_DISTANCE;
    }

    private boolean outakeDetectsNote() {
        return getGamePieceDistanceOutake() < DETECT_DISTANCE;
    }
    //TODO replace pidControllerIntake.setReference with the new method
    
    public void senseGamePieceStop() {//This slows and stops the motors when the distance sensor detects the notes
        if (intakeDetectsNote() && !outakeDetectsNote() ) {
            setRPMIntake(-1.0);
        }
        if (outakeDetectsNote() ) {
            setRPMIntake(0.0);
            }
        }

    //Aaron will work on this
    public boolean noteInIntake(){
        return intakeDetectsNote() && outakeDetectsNote();
    }    
    // //Find offset of note from the center line using big mathy mathy, god I hope this works chatgpt gave me the formulas :))))))
    // //find out what this means
    // public double calculateDistanceSensorNotes() {
    //     double center = 11.485;// center line between the 2 side plates (in)
    //     double d1 = getGamePieceDistanceIntake();
    //     double d2 = getGamePieceDistanceOutake();
    //     double r = 7;
    //     double ym = (d1+d2)/2; //Y midpoint between 2 points
    //     double k = ym + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2, 2)) * (DISTANCE_BETWEEN_SENSORS))/r;// y cord of center
    //     //Take into note that in reality, the 2 points can return 2 possible centers
    //     return k - center; //<- offset from the center
    // }
    // //find out what this means
    // public double calculateIntakeAmount(){
    //     //Literatly just calcDistanceSensorNotes but instead of solving for k, we are solving for h
    //     double d1 = getGamePieceDistanceIntake();
    //     double d2 = getGamePieceDistanceOutake();
    //     double r = 7;

    //     double xm = (DISTANCE_BETWEEN_SENSORS)/2;

    //     double h = xm + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2,2)) * (d1-d2))/r; 
    //     return h;
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Outake Velocity", outakeEncoder.getVelocity());
        SmartDashboard.putNumber("Intake Velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("distance sensor 1", getGamePieceDistanceIntake());
        SmartDashboard.putNumber("distance sensor 2", getGamePieceDistanceOutake());
        SmartDashboard.putBoolean("DS1 Sees piece", intakeDetectsNote());
        SmartDashboard.putBoolean("DS2 Sees piece", outakeDetectsNote());
        senseGamePieceStop();// slows down when sensed by the first Sensor and stops upon being sensed by the second
    }
    public void setRPMOutake(double rpm) {
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(rpm/60));
    }
    public void setRPMIntake(double rpm) {
        pidControllerIntake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(rpm/60));
    }
    //TODO create a method that checks if the shooting rpm is high enough and commands can use
    public double getOutakeRPM(){
        double RPM = outakeEncoder.getVelocity();
        return RPM;
    }
    public void stopOutake() {
        setRPMOutake(0);
    }
    

    public void stopIntake() {
        setRPMIntake(0);
    }
    /* 
    public double calculateRPMAtDistance() {

        double minRPM = Integer.MAX_VALUE;
        double distance = limelight.distanceToTargetSpeaker(); // placeholder for limelight 
        for(int i = 0; i<= 360; i++) {
            double t = Math.sqrt((OFFSETFROMGROUND-SPEAKER_HEIGHT+distance*Math.tan(i)));
            double rpm = distance/Math.cos(i)*t;
            if(rpm<minRPM) {
                minRPM = rpm;
            }
        }   
        if(minRPM == Integer.MAX_VALUE) {
            System.err.println("FAILURE");
        }
        return minRPM;
    }
*/
}
