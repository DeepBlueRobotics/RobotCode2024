package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(INTAKE_PORT, MotorConfig.NEO_550);
    // private final CANSparkMax outakeMotor =
    // MotorControllerFactory.createSparkMax(10, MotorConfig.NEO_550);
    private final CANSparkFlex outakeMotorVortex = new CANSparkFlex(10, MotorType.kBrushless);
    private final RelativeEncoder outakeEncoder = outakeMotorVortex.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final SparkPIDController pidControllerOutake = outakeMotorVortex.getPIDController();
    private final SparkPIDController pidControllerIntake = intakeMotor.getPIDController();
    private Timer timer = new Timer();
    private int count = 0;

    private final SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(kS[INTAKE], kV[INTAKE],
            kA[INTAKE]);
    private final SimpleMotorFeedforward outakeFeedforward = new SimpleMotorFeedforward(kS[OUTTAKE], kV[OUTTAKE],
            kA[OUTTAKE]);

    private double goalOutakeRPM = outakeEncoder.getVelocity();

    private TimeOfFlight intakeDistanceSensor = new TimeOfFlight(INTAKE_DISTANCE_SENSOR_PORT);
    private TimeOfFlight OutakeDistanceSensor = new TimeOfFlight(OUTAKE_DISTANCE_SENSOR_PORT);
    
    public IntakeShooter() {
        // Figure out which ones to set inverted
        intakeMotor.setInverted(INTAKE_MOTOR_INVERSION);
        outakeMotorVortex.setInverted(OUTAKE_MOTOR_INVERSION);
        pidControllerOutake.setP(kP[OUTTAKE]);
        pidControllerOutake.setI(kI[OUTTAKE]);
        pidControllerOutake.setD(kD[OUTTAKE]);
        pidControllerIntake.setP(kP[INTAKE]);
        pidControllerIntake.setI(kI[INTAKE]);
        pidControllerIntake.setD(kD[INTAKE]);
        SmartDashboard.putData("Intake Shooter", this);
        intakeEncoder.setAverageDepth(4);
        intakeEncoder.setMeasurementPeriod(8);
        SmartDashboard.putNumber("intake volts", 0);
        SmartDashboard.putNumber("Vortex volts", 0);
        // setMaxOutakeOverload(1);
        outakeMotorVortex.setSmartCurrentLimit(60);
        

    }

    // ---------------------------------------------------------------------------------------------------
    // checking whether RPM is within tolerance
    public boolean isWithinTolerance() {
        return outakeEncoder.getVelocity() < goalOutakeRPM + RPM_TOLERANCE
                && goalOutakeRPM - RPM_TOLERANCE < outakeEncoder.getVelocity();
    }

    private double countPeridoic() {
        return count / timer.get();
    }

    // ---------------------------------------------------------------------------------------------------
    private double getGamePieceDistanceIntake() {
        return Units.metersToInches(intakeDistanceSensor.getRange() / 1000) - DS_DEPTH_INCHES;
    }

    public void motorSetOutake(int speed) {
        outakeMotorVortex.set(speed);
    }

    private double getGamePieceDistanceOutake() {
        return Units.metersToInches(OutakeDistanceSensor.getRange() / 1000) - DS_DEPTH_INCHES;
    }

    public boolean intakeDetectsNote() {
        return getGamePieceDistanceIntake() < DETECT_DISTANCE_INCHES;
    }

    public boolean outakeDetectsNote() {
        return getGamePieceDistanceOutake() < DETECT_DISTANCE_INCHES;
    }

    // Aaron will work on this

    // //Find offset of note from the center line using big mathy mathy, god I hope
    // this works chatgpt gave me the formulas :))))))
    // //find out what this means
    // public double calculateDistanceSensorNotes() {
    // double center = 11.485;// center line between the 2 side plates (in)
    // double d1 = getGamePieceDistanceIntake();
    // double d2 = getGamePieceDistanceOutake();
    // double r = 7;
    // double ym = (d1+d2)/2; //Y midpoint between 2 points
    // double k = ym + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2, 2)) *
    // (DISTANCE_BETWEEN_SENSORS))/r;// y cord of center
    // //Take into note that in reality, the 2 points can return 2 possible centers
    // return k - center; //<- offset from the center
    // }
    // //find out what this means
    // public double calculateIntakeAmount(){
    // //Literatly just calcDistanceSensorNotes but instead of solving for k, we are
    // solving for h
    // double d1 = getGamePieceDistanceIntake();
    // double d2 = getGamePieceDistanceOutake();
    // double r = 7;

    // double xm = (DISTANCE_BETWEEN_SENSORS)/2;

    // double h = xm + (Math.sqrt(Math.pow(r,2) - Math.pow(r/2,2)) * (d1-d2))/r;
    // return h;
    // }

    @Override
    public void periodic() {
        // outakeMotor.set(SmartDashboard.getNumber("intake volts", 0));
        //intakeMotor.set(SmartDashboard.getNumber("intake volts", 0));
        SmartDashboard.putNumber("outtake vel", outakeMotorVortex.getEncoder().getVelocity());

        // count++;
        SmartDashboard.putBoolean("Intake detects note", intakeDetectsNote());
        SmartDashboard.putBoolean("Intake detects note", outakeDetectsNote());

        double volts = SmartDashboard.getNumber("Vortex volts", 0);
       //outakeMotorVortex.set(volts);

        // setMaxOutake();


    }

    public void driveMotor(double volts) {
        // outakeMotorVortex.set(volts);
    }

    public void setCurrentLimit(int limit) {
        intakeMotor.setSmartCurrentLimit(limit);
    }

    public void setMaxIntake(int direction) {
        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.set(1 * direction);

    }

    public void setMaxOutakeOverload(int direction) {
        // outakeMotor.setSmartCurrentLimit(40);
        // outakeMotor.setSmartCurrentLimit(1*direction);
    }

    public void setMaxOutake() {
        outakeMotorVortex.set(1);
    }

    public void resetCurrentLimit() {
        intakeMotor.setSmartCurrentLimit(MotorConfig.NEO_550.currentLimitAmps);
        // outakeMotorVortex.setSmartCurrentLimit(MotorConfig.NEO.currentLimitAmps);
        // intakeMotor.setSmartCurrentLimit(MotorConfig.NEO_550.currentLimitAmps);
    }

    public void setRPMOutake(double rpm) {
        pidControllerOutake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0,
                outakeFeedforward.calculate(rpm / 60.0));
    }

    public void setRPMIntake(double rpm) {
        pidControllerIntake.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0,
                intakeFeedforward.calculate(rpm / 60.0));
    }

    public double getOutakeRPM() {
        return outakeEncoder.getVelocity();
    }

    public void stopOutake() {
        setRPMOutake(0);
    }

    public void stopIntake() {
        intakeMotor.set(0);
        outakeMotorVortex.set(0);
    }

    public double getIntakeRPM() {
        return intakeEncoder.getVelocity();
    }

    public double getVortexRPM() {
        return 0;
        // return outakeMotorVortex.getEncoder().getVelocity();
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.addDoubleProperty("Outtake Velocity", this::getOutakeRPM, null);
        sendableBuilder.addDoubleProperty("Intake velocity", this::getIntakeRPM, null);
        sendableBuilder.addDoubleProperty("Outake distance sensor", this::getGamePieceDistanceOutake, null);
        sendableBuilder.addDoubleProperty("Intake distance sensor", this::getGamePieceDistanceIntake, null);
        sendableBuilder.addBooleanProperty("Outake distance sensor length", this::outakeDetectsNote, null);
        sendableBuilder.addBooleanProperty("Intake distance sensor length", this::intakeDetectsNote, null);
        sendableBuilder.addDoubleProperty("Period time", this::countPeridoic, null);
        sendableBuilder.addDoubleProperty("Vortex Motor Velocity", this::getVortexRPM, null);
    }
    /*
     * public double calculateRPMAtDistance() {
     * 
     * double minRPM = Integer.MAX_VALUE;
     * double distance = limelight.distanceToTargetSpeaker(); // placeholder for
     * limelight
     * for(int i = 0; i<= 360; i++) {
     * double t = Math.sqrt((OFFSETFROMGROUND-SPEAKER_HEIGHT+distance*Math.tan(i)));
     * double rpm = distance/Math.cos(i)*t;
     * if(rpm<minRPM) {
     * minRPM = rpm;
     * }
     * }
     * if(minRPM == Integer.MAX_VALUE) {
     * System.err.println("FAILURE");
     * }
     * return minRPM;
     * }
     */
}
