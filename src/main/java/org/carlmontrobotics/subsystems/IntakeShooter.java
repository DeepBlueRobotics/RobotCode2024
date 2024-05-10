package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(INTAKE_PORT, MotorConfig.NEO);
    // private final CANSparkMax outakeMotor =
    // MotorControllerFactory.createSparkMax(10, MotorConfig.NEO_550);
    private final CANSparkFlex outakeMotorVortex = new CANSparkFlex(10, MotorType.kBrushless);
    private final RelativeEncoder outakeEncoder = outakeMotorVortex.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final SparkPIDController pidControllerOutake = outakeMotorVortex.getPIDController();
    private final SparkPIDController pidControllerIntake = intakeMotor.getPIDController();
    private Timer timer = new Timer();
    private Timer intakeTOFTimer = new Timer();
    private Timer outtakeTOFTimer = new Timer();
    private int count = 0;
    private StringLogEntry tofLogEntry;
    private SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(kS[INTAKE], kV[INTAKE],
            kA[INTAKE]);
    private final SimpleMotorFeedforward outakeFeedforward = new SimpleMotorFeedforward(kS[OUTTAKE], kV[OUTTAKE],
            kA[OUTTAKE]);

    private double goalOutakeRPM = outakeEncoder.getVelocity();

    private TimeOfFlight intakeDistanceSensor = new TimeOfFlight(INTAKE_DISTANCE_SENSOR_PORT);
    private TimeOfFlight OutakeDistanceSensor = new TimeOfFlight(OUTAKE_DISTANCE_SENSOR_PORT);

    private double lastValidDistanceIntake = Double.POSITIVE_INFINITY;
    private double lastValidDistanceOuttake = Double.POSITIVE_INFINITY;

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
        SmartDashboard.putNumber("Intake Ks", kS[INTAKE]);
        SmartDashboard.putNumber("Intake Kv", kV[INTAKE]);
        intakeEncoder.setAverageDepth(4);
        intakeEncoder.setMeasurementPeriod(8);
        // SmartDashboard.putNumber("intake volts", 0);
        // SmartDashboard.putNumber("Vortex volts", 0);
        // setMaxOutakeOverload(1);
        intakeDistanceSensor.setRangingMode(RangingMode.Short, 24);// 24 ms is the minimum sample time acc to docs
        OutakeDistanceSensor.setRangingMode(RangingMode.Short, 24);
        outakeMotorVortex.setSmartCurrentLimit(60);
        SmartDashboard.putNumber("Intake target RPM", 0);
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

    public void motorSetOutake(double speed) {
        outakeMotorVortex.set(speed);
    }

    public void motorSetIntake(double speed) {
        intakeMotor.set(speed);
    }

    private double getGamePieceDistanceIntake() {
        // return Units.metersToInches(intakeDistanceSensor.getRange() / 1000) -
        // DS_DEPTH_INCHES;
        return lastValidDistanceIntake;
    }

    private double getGamePieceDistanceOutake() {
        // return Units.metersToInches(OutakeDistanceSensor.getRange() / 1000) -
        // DS_DEPTH_INCHES;
        return lastValidDistanceOuttake;
    }

    public boolean intakeDetectsNote() {
        return getGamePieceDistanceIntake() < DETECT_DISTANCE_INCHES;
    }

    public boolean outakeDetectsNote() {
        return getGamePieceDistanceOutake() < DETECT_DISTANCE_INCHES;
    }

    public void updateValues() {
        if (intakeDistanceSensor.isRangeValid()) {
            lastValidDistanceIntake = Units.metersToInches(intakeDistanceSensor.getRange()) / 1000.0;
            if (lastValidDistanceIntake != Double.POSITIVE_INFINITY) {
                tofLogEntry.append("Time between valid ranges intake " + intakeTOFTimer.get());
                intakeTOFTimer.reset();
            } else
                intakeTOFTimer.start();
        }
        if (OutakeDistanceSensor.isRangeValid()) {
            lastValidDistanceOuttake = Units.metersToInches(OutakeDistanceSensor.getRange()) / 1000.0;
            if (lastValidDistanceOuttake != Double.POSITIVE_INFINITY) {
                tofLogEntry.append("Time between valid ranges outtake " + outtakeTOFTimer.get());
                outtakeTOFTimer.reset();
            } else
                outtakeTOFTimer.start();
        }
    }

    @Override
    public void periodic() {
        updateValues();
        double newKS = SmartDashboard.getNumber("Intake Ks", kS[INTAKE]);
        double newKV = SmartDashboard.getNumber("Intake Kv", kV[INTAKE]);

        if (newKS != intakeFeedforward.ks || newKV != intakeFeedforward.kv) {
            intakeFeedforward = new SimpleMotorFeedforward(newKS, newKV);
        }
        setRPMIntake(SmartDashboard.getNumber("Intake target RPM", 0));
        SmartDashboard.putBoolean("instake ds sees", intakeDetectsNote());
        SmartDashboard.putBoolean("outtake ds sees", outakeDetectsNote());
        SmartDashboard.putNumber("intake sample rate", intakeDistanceSensor.getSampleTime());
        SmartDashboard.putData("intake distanace sensor", intakeDistanceSensor);
        SmartDashboard.putBoolean("intake ds range valid", intakeDistanceSensor.isRangeValid());
        SmartDashboard.putData("outtake distanace sensor", OutakeDistanceSensor);
        SmartDashboard.putBoolean("outtake ds range valid", OutakeDistanceSensor.isRangeValid());
        SmartDashboard.putNumber("Intake Vel", intakeEncoder.getVelocity());
        TimeOfFlight.RangingMode rangingModeIntake = intakeDistanceSensor.getRangingMode();
        if (rangingModeIntake == TimeOfFlight.RangingMode.Long)
            SmartDashboard.putString("intake ds ranging mode", "long");
        else if (rangingModeIntake == TimeOfFlight.RangingMode.Medium)
            SmartDashboard.putString("intake ds ranging mode", "medium");
        else
            SmartDashboard.putString("intake ds ranging mode", "short");

        TimeOfFlight.RangingMode rangingModeOuttake = OutakeDistanceSensor.getRangingMode();
        if (rangingModeOuttake == TimeOfFlight.RangingMode.Long)
            SmartDashboard.putString("outake ds ranging mode", "long");
        else if (rangingModeOuttake == TimeOfFlight.RangingMode.Medium)
            SmartDashboard.putString("outake ds ranging mode", "medium");
        else
            SmartDashboard.putString("outake ds ranging mode", "short");
        // intakeMotor.set(0.1);
        // outakeMotor.set(SmartDashboard.getNumber("intake volts", 0));

        // count++;

        // double volts = SmartDashboard.getNumber("Vortex volts", 0);
        // outakeMotorVortex.set(volts);

        // setMaxOutake();

        SmartDashboard.putNumber("Intake amps", intakeMotor.getOutputCurrent());

        if (intakeIsOverTemp()) {
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
