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
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(INTAKE_PORT, MotorConfig.NEO);
    // private final CANSparkMax outakeMotor = MotorControllerFactory.createSparkMax(10, MotorConfig.NEO_550);
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
        // SmartDashboard.putNumber("intake volts", 0);
        // SmartDashboard.putNumber("Vortex volts", 0);
        // setMaxOutakeOverload(1);
        outakeMotorVortex.setSmartCurrentLimit(60);
        
        

    }
    public boolean intakeIsOverTemp() {
        return intakeMotor.getMotorTemperature() >= MotorConfig.NEO.temperatureLimitCelsius;
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
    public void motorSetIntake(double speed) {
        intakeMotor.set(speed);
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

    

    @Override
    public void periodic() {
    //intakeMotor.set(0.1);
        // outakeMotor.set(SmartDashboard.getNumber("intake volts", 0));

        // count++;

        // double volts = SmartDashboard.getNumber("Vortex volts", 0);
       //outakeMotorVortex.set(volts);

        // setMaxOutake();

        SmartDashboard.putNumber("Intake amps", intakeMotor.getOutputCurrent());

        if (intakeIsOverTemp()){
            turnOffIntakeMotor();
            stopIntake();
            System.err.println("INTAKE IS OVER TEMP!!!!\nBIG BAD\nOOPSY WOOPSY\nTURN IT OFF");
        }
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

   

    public void setMaxOutake(int direction) {
        outakeMotorVortex.set(1 * direction);
    }
    public void setMaxOutakeOverload(int direction) {
        outakeMotorVortex.setSmartCurrentLimit(80);
        outakeMotorVortex.set(1 * direction);
    }
    public void resetCurrentLimit() {
        intakeMotor.setSmartCurrentLimit(MotorConfig.NEO.currentLimitAmps);
        outakeMotorVortex.setSmartCurrentLimit(60);
        
        // intakeMotor.setSmartCurrentLimit(MotorConfig.NEO_550.currentLimitAmps);
    }
    public void turnOffIntakeMotor() {
        intakeMotor.setSmartCurrentLimit(1);
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
       
        return outakeMotorVortex.getEncoder().getVelocity();
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        super.initSendable(sendableBuilder);
        sendableBuilder.addDoubleProperty("Outtake Velocity", this::getOutakeRPM, null);
        sendableBuilder.addDoubleProperty("Intake velocity", this::getIntakeRPM, null);
        sendableBuilder.addDoubleProperty("Outake distance sensor", this::getGamePieceDistanceOutake, null);
        sendableBuilder.addDoubleProperty("Intake distance sensor", this::getGamePieceDistanceIntake, null);
        sendableBuilder.addBooleanProperty("Outake distance sensor length", this::outakeDetectsNote, null);
        sendableBuilder.addBooleanProperty("Intake distance sensor length", this::intakeDetectsNote, null);
        sendableBuilder.addDoubleProperty("Period time", this::countPeridoic, null);
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
