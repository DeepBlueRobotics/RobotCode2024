package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.IntakeShoot.*;
import static org.mockito.ArgumentMatchers.matches;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

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
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(intakePort, MotorConfig.NEO_550);
    private final CANSparkMax outakeMotor = MotorControllerFactory.createSparkMax(outakePort, MotorConfig.NEO_550);
    private final RelativeEncoder outakeEncoder = outakeMotor.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final SparkPIDController pidControllerOutake = outakeMotor.getPIDController();
    private final SparkPIDController pidControllerIntake = intakeMotor.getPIDController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);//for both intake and outtake 
    private TimeOfFlight distanceSensor = new TimeOfFlight(distanceSensorPort1); // make sure id port is correct here
    private TimeOfFlight distanceSensor2 = new TimeOfFlight(distanceSensorPort2); // insert\
    
    public IntakeShooter() {
        //Figure out which ones to set inverted 
        intakeMotor.setInverted(false);
        outakeMotor.setInverted(true);         
        pidControllerOutake.setP(kP[0]);
        pidControllerOutake.setI(kI[0]);
        pidControllerOutake.setD(kD[0]);
        pidControllerIntake.setP(kP[1]);
        pidControllerIntake.setI(kI[1]);
        pidControllerIntake.setD(kD[1]);
    }
    public boolean isWithinTolerance(double outakeRPM){
        return outakeEncoder.getVelocity()<outakeRPM+RPM_TOLERANCE && outakeRPM-RPM_TOLERANCE<outakeEncoder.getVelocity();
    }
    public double getGamePieceDistanceIntake() {
        return Units.metersToInches((distanceSensor.getRange() - dsDepth) / 1000);
    }

    public double getGamePieceDistanceOutake() {
        return Units.metersToInches((distanceSensor2.getRange() - dsDepth) / 1000);
    }

    public boolean gameDistanceSees1st() {
        return getGamePieceDistanceIntake() < detectDistance;
    }

    public boolean gameDistanceSees2nd() {
        return getGamePieceDistanceOutake() < detectDistance;
    }
    public void senseGamePieceStop() {//This slows and stops the motors when the distance sensor detects the notes
        if (gameDistanceSees1st()) {
            pidControllerIntake.setReference((-1), CANSparkBase.ControlType.kVelocity, 0,
                    feedforward.calculate(-1 / 60));//Slows down the motors once the first distance sensor detects the note
            if (gameDistanceSees2nd()) {
                pidControllerIntake.setReference(0, CANSparkBase.ControlType.kVelocity, 0, 
                        feedforward.calculate(0));//Stops it when the second distance sensor detects the note
            }
        }
    }
    public boolean noNote(){
        return ( !gameDistanceSees1st() && !gameDistanceSees2nd() );
    }
    public boolean noteInMiddle(){
        return gameDistanceSees1st() && gameDistanceSees2nd();
    }
    public boolean noteInIntake(){
        return gameDistanceSees1st() && !gameDistanceSees2nd();
    }    
    public boolean noteInOutake(){    
        return !gameDistanceSees1st() && gameDistanceSees2nd();
    }
    //Find offset of note from the center line using big mathy mathy, god I hope this works chatgpt gave me the formulas :))))))
    public double calculateDistanceSensorNotes() {
        double center = 11.485;// center line between the 2 side plates (in)
        double d1 = getGamePieceDistanceIntake();
        double d2 = getGamePieceDistanceOutake();
        double r = 7;
        double ym = (d1+d2)/2; //Y midpoint between 2 points
        double k = ym + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2, 2)) * (distanceBetweenSensors))/r;// y cord of center
        //Take into note that in reality, the 2 points can return 2 possible centers
        return k - center; //<- offset from the center
    }

    public double calculateIntakeAmount(){
        //Literatly just calcDistanceSensorNotes but instead of solving for k, we are solving for h
        double d1 = getGamePieceDistanceIntake();
        double d2 = getGamePieceDistanceOutake();
        double r = 7;

        double xm = (distanceBetweenSensors)/2;

        double h = xm + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2,2)) * (d1-d2))/r; 
        return h;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Outake Velocity", outakeEncoder.getVelocity());
        SmartDashboard.putNumber("Intake Velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("distance sensor 1", getGamePieceDistanceIntake());
        SmartDashboard.putNumber("distance sensor 2", getGamePieceDistanceOutake());
        SmartDashboard.putBoolean("DS1 Sees piece", gameDistanceSees1st());
        SmartDashboard.putBoolean("DS2 Sees piece", gameDistanceSees2nd());
        senseGamePieceStop();// slows down when sensed by the first Sensor and stops upon being sensed by the second
    }
    public void setRPMOutake(double rpm) {
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(rpm/60));
    }

    public void setRPMintake(double rpm) {
        pidControllerIntake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(rpm/60));
    }



    public void shoot(double distance) {
        double rpm = calculateRPMAtDistance();
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(rpm/60));
    }

    public void stopOutake() {
        setRPMOutake(0);
    }

    public void stopIntake() {
        setRPMintake(0);
    }

    public double calculateRPMAtDistance() {
        double minRPM = Integer.MAX_VALUE;
        double distance = 30; // placeholder for limelight 
        for(int i = 0; i<= 360; i++) {
            double t = Math.sqrt((OFFSETFROMGROUND-SpeakerHeight+distance*Math.tan(i)));
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

}
