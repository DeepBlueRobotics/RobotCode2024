package org.carlmontrobotics.subsystems;
 

import static org.carlmontrobotics.Constants.IntakeShoot.DETECT_DISTANCE_INCHES;
import static org.carlmontrobotics.Constants.IntakeShoot.DS_DEPTH_INCHES;
import static org.carlmontrobotics.Constants.IntakeShoot.INTAKE;
import static org.carlmontrobotics.Constants.IntakeShoot.INTAKE_DISTANCE_SENSOR_PORT;
import static org.carlmontrobotics.Constants.IntakeShoot.INTAKE_MOTOR_INVERSION;
import static org.carlmontrobotics.Constants.IntakeShoot.INTAKE_PORT;
import static org.carlmontrobotics.Constants.IntakeShoot.OUTAKE_DISTANCE_SENSOR_PORT;
import static org.carlmontrobotics.Constants.IntakeShoot.OUTAKE_MOTOR_INVERSION;
import static org.carlmontrobotics.Constants.IntakeShoot.OUTAKE_PORT;
import static org.carlmontrobotics.Constants.IntakeShoot.OUTTAKE;
import static org.carlmontrobotics.Constants.IntakeShoot.RPM_TOLERANCE;
import static org.carlmontrobotics.Constants.IntakeShoot.defaultColor;
import static org.carlmontrobotics.Constants.IntakeShoot.kA;
import static org.carlmontrobotics.Constants.IntakeShoot.kD;
import static org.carlmontrobotics.Constants.IntakeShoot.kI;
import static org.carlmontrobotics.Constants.IntakeShoot.kP;
import static org.carlmontrobotics.Constants.IntakeShoot.kS;
import static org.carlmontrobotics.Constants.IntakeShoot.kV;
import static org.carlmontrobotics.Constants.IntakeShoot.ledDefaultColorRestoreTime;
import static org.carlmontrobotics.Constants.IntakeShoot.pickupSuccessColor;

import org.carlmontrobotics.Constants;

import static org.carlmontrobotics.Constants.IntakeShoot.ledLength;
import static org.carlmontrobotics.Constants.IntakeShoot.ledPort;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class IntakeShooter extends SubsystemBase {
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(INTAKE_PORT, MotorConfig.NEO_550);
    private final CANSparkMax outakeMotor = MotorControllerFactory.createSparkMax(OUTAKE_PORT, MotorConfig.NEO_550);
    private final RelativeEncoder outakeEncoder = outakeMotor.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final SparkPIDController pidControllerOutake = outakeMotor.getPIDController();
    private final SparkPIDController pidControllerIntake = intakeMotor.getPIDController();
    private final SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(kS[INTAKE], kV[INTAKE], kA[INTAKE]);//for both intake and outtake 
    private final SimpleMotorFeedforward outakeFeedforward = new SimpleMotorFeedforward(kS[OUTTAKE], kV[OUTTAKE], kA[OUTTAKE]);//for both intake and outtake 
    private TimeOfFlight intakeDistanceSensor = new TimeOfFlight(INTAKE_DISTANCE_SENSOR_PORT); // make sure id port is correct here
    private TimeOfFlight OutakeDistanceSensor = new TimeOfFlight(OUTAKE_DISTANCE_SENSOR_PORT); // insert
    private double goalOutakeRPM = outakeEncoder.getVelocity();
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);

    
    public IntakeShooter() {
        //Figure out which ones to set inverted
        intakeMotor.setInverted(INTAKE_MOTOR_INVERSION);
        outakeMotor.setInverted(OUTAKE_MOTOR_INVERSION);         
        pidControllerOutake.setP(kP[OUTTAKE]);
        pidControllerOutake.setI(kI[OUTTAKE]);
        pidControllerOutake.setD(kD[OUTTAKE]);
        pidControllerIntake.setP(kP[INTAKE]);
        pidControllerIntake.setI(kI[INTAKE]);
        pidControllerIntake.setD(kD[INTAKE]);
        intakeMotor.setSmartCurrentLimit(20);
    }

    //---------------------------------------------------------------------------------------------------
    //checking whether RPM is within tolerance
    public boolean isWithinTolerance(){
        return outakeEncoder.getVelocity()<goalOutakeRPM+RPM_TOLERANCE && goalOutakeRPM-RPM_TOLERANCE<outakeEncoder.getVelocity();
    }
    //---------------------------------------------------------------------------------------------------
    //TODO: fix the unit conversion
    private double getGamePieceDistanceIntake() {
        return Units.metersToInches(intakeDistanceSensor.getRange()/1000/*mm->m*/)- DS_DEPTH_INCHES;
    }

    private double getGamePieceDistanceOutake() {
        return Units.metersToInches((OutakeDistanceSensor.getRange() - DS_DEPTH_INCHES) / 1000);
    }

    public boolean intakeDetectsNote() {
        return getGamePieceDistanceIntake() < DETECT_DISTANCE_INCHES;

    }

    public boolean outakeDetectsNote() {
        return getGamePieceDistanceOutake() < DETECT_DISTANCE_INCHES;

    }
    
    //Aaron will work on this
      
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
        SmartDashboard.putBoolean("intakeDetctsNote", intakeDetectsNote());
        SmartDashboard.putBoolean("outakeDetctsNote", outakeDetectsNote());
        SmartDashboard.putNumber("Outake Velocity", outakeEncoder.getVelocity());
        SmartDashboard.putNumber("Intake Velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("distance sensor intake", getGamePieceDistanceIntake());
        SmartDashboard.putNumber("distance sensor outake", getGamePieceDistanceOutake());
        SmartDashboard.putBoolean("DSIntake Sees piece", intakeDetectsNote());
        SmartDashboard.putBoolean("DSOutake Sees piece", outakeDetectsNote());
       

    }
     
    public void setCurrentLimit(int limit) {
        intakeMotor.setSmartCurrentLimit(limit);
    }
    public void setMaxIntake() {
        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.set(-1);
        
    }

    public void setRPMOutake(double rpm) {
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, outakeFeedforward.calculate(rpm/60.0));
    }

    public void setRPMIntake(double rpm) {
        pidControllerIntake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, intakeFeedforward.calculate(rpm/60.0));
    }

    public double getOutakeRPM(){
        return outakeEncoder.getVelocity();
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
