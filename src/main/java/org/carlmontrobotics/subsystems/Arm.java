package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Armc.*;

import org.carlmontrobotics.commands.TeleopArm;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.opencv.core.Mat;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// Arm angle is measured from horizontal on the intake side of the robot and bounded between -3π/2 and π/2
public class Arm extends SubsystemBase {

    private final CANSparkMax armMotorMaster/* left */ = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT_MASTER,
            MotorConfig.NEO);
    private final CANSparkMax armMotorFollower/* right */ = MotorControllerFactory
            .createSparkMax(ARM_MOTOR_PORT_FOLLOWER, MotorConfig.NEO);
    private final SparkAbsoluteEncoder armMasterEncoder = armMotorMaster
            .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private static double kDt = 0.02;//what is this?

    // PID, feedforward, trap profile

    // rel offset = starting absolute offset
    private double armFeedVolts;//for SendableBuilder
    private final ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
    private final SparkPIDController armPIDMaster = armMotorMaster.getPIDController();
    private static TrapezoidProfile.State setpoint;

    private TrapezoidProfile armProfile;
    private TrapezoidProfile.State goalState;

    private double lastMeasuredTime;
    private double lastArmPos;

    private boolean isArmEncoderConnected = false;

    public Arm() {
        // weird math stuff
        armMotorMaster.setInverted(MOTOR_INVERTED_MASTER);
        armMotorMaster.setIdleMode(IdleMode.kBrake);
        armMotorFollower.setInverted(MOTOR_INVERTED_FOLLOWER);
        armMotorFollower.setIdleMode(IdleMode.kBrake);
        // Comment out when running sysid
        armMasterEncoder.setPositionConversionFactor(ROTATION_TO_RAD);
        armMasterEncoder.setVelocityConversionFactor(ROTATION_TO_RAD);
        armMasterEncoder.setZeroOffset(ENCODER_OFFSET_RAD);
        // ------------------------------------------------------------
        armMasterEncoder.setInverted(ENCODER_INVERTED);

        armMotorFollower.follow(armMotorMaster, MOTOR_INVERTED_FOLLOWER);

        // //SmartDashboard.putNumber("set KP", kP);
        // //armPIDMaster.setP(SmartDashboard.getNumber("set KP", kP));
        // SmartDashboard.putNumber("set KP", kP);
        // armPIDMaster.setP(SmartDashboard.getNumber("set KP", kP));
        // armPIDMaster.setI(kI);
        // SmartDashboard.putNumber("set kD", kD);
        // armPIDMaster.setD(SmartDashboard.getNumber("set kD", kD));
        // SmartDashboard.putNumber("set kG", kG);
        // // armPIDMaster.setD(SmartDashboard.getNumber("set kD", kD));
        // armPIDMaster.setOutputRange(MIN_VOLTAGE/12, MAX_VOLTAGE/12);
        // // armPID.setTolerance(posToleranceRad, velToleranceRadPSec);

        armPIDMaster.setFeedbackDevice(armMasterEncoder);
        armPIDMaster.setPositionPIDWrappingEnabled(true);
        armPIDMaster.setPositionPIDWrappingMinInput(-Math.PI/2);
        armPIDMaster.setPositionPIDWrappingMaxInput((3*Math.PI)/ 2);
        armPIDMaster.setIZone(IZONE_RAD);

        TRAP_CONSTRAINTS = new TrapezoidProfile.Constraints(
            Math.PI,
            MAX_FF_ACCEL_RAD_P_S);
        //^ worst case scenario
        //armFeed.maxAchievableVelocity(12, 0, MAX_FF_ACCEL_RAD_P_S)
        armProfile = new TrapezoidProfile(TRAP_CONSTRAINTS);

        SmartDashboard.putData("Arm", this);

        setpoint = getCurrentArmState();
        goalState = getCurrentArmState();
        setArmTarget(goalState.position);

        lastArmPos = getArmPos();
        lastMeasuredTime = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled())
            resetGoal();

        // ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD =
        // SmartDashboard.getNumber("ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD",
        // ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);
        // armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL , MAX_FF_ACCEL
        // );

        //SmartDashboard.putNumber("KP", );
        // smart dahsboard stuff
        SmartDashboard.putBoolean("ArmPIDAtSetpoint", armAtSetpoint());
        SmartDashboard.putNumber("Arm Goal Pos (rad)", goalState.position);
        // SmartDashboard.putBoolean("ArmProfileFinished",
        // armProfile.isFinished(armProfileTimer.get()));
        // posToleranceRad = SmartDashboard.getNumber("Arm Tolerance Pos",
        // posToleranceRad);
        // velToleranceRadPSec= SmartDashboard.getNumber("Arm Tolerance Vel",
        // velToleranceRadPSec);

        SmartDashboard.putNumber("InternalArmVelocity", armMasterEncoder.getVelocity());
        // SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());

        // SmartDashboard.putNumber("ArmPos", getArmPos());
        double currTime = Timer.getFPGATimestamp();
        // SmartDashboard.putNumber("Current Time", currTime);
        //SmartDashboard.putNumber("Last Update (s)", lastMeasuredTime);
        setArmTarget(SmartDashboard.getNumber("set arm angle (rad)", 0));

        double currP = SmartDashboard.getNumber("set kP", kP);
        double KP = kP;
        if (currP != KP) {
            armPIDMaster.setP(SmartDashboard.getNumber("set KP", kP));
            KP = currP;

        }

        double currD = SmartDashboard.getNumber("set kD", kD);
        double KD = kD;
        if (currD != KD) {
            armPIDMaster.setD(SmartDashboard.getNumber("set kD", currD));
            KD = currD;

        }

        // double currG = SmartDashboard.getNumber("set kG", kG);
        // double KG = kG;
        // if (currG != KG) {
        //     armFeed = new ArmFeedforward(kS, SmartDashboard.getNumber("set kG", currG), kV, kA);
        //     KG = currG;
        // }

        // when the value is different
        double currentArmPos = getArmPos();
        if (currentArmPos != lastArmPos) {
            lastMeasuredTime = currTime;
            lastArmPos = currentArmPos;
        }
        isArmEncoderConnected = currTime - lastMeasuredTime < DISCONNECTED_ENCODER_TIMEOUT_SEC;

        if (isArmEncoderConnected){
            driveArm();
        }
        else {
            armMotorMaster.set(0);
            armMotorFollower.set(0);
        }
        autoCancelArmCommand();

    }

    public void autoCancelArmCommand() {
        if (!(getDefaultCommand() instanceof TeleopArm) || DriverStation.isAutonomous())
            return;

        double requestedSpeeds = ((TeleopArm) getDefaultCommand()).getRequestedSpeeds();

        if (requestedSpeeds != 0) {
            Command currentArmCommand = getCurrentCommand();
            if (currentArmCommand != getDefaultCommand() && currentArmCommand != null) {
                currentArmCommand.cancel();
            }
        }
    }

    // #region Drive Methods
    private void driveArm() {
        setpoint = armProfile.calculate(kDt, setpoint, goalState);
        SmartDashboard.putNumber("setpoint goal (rad)", setpoint.position);
        armFeedVolts = armFeed.calculate(setpoint.position, setpoint.velocity);
        if ((getArmPos() < LOWER_ANGLE_LIMIT_RAD)
                || (getArmPos() > UPPER_ANGLE_LIMIT_RAD)) {
            armFeedVolts = armFeed.calculate(getArmPos(), 0);
            //kg * cos(arm angle) * arm_COM_length
        }
        armPIDMaster.setReference(Units.radiansToRotations(setpoint.position), CANSparkBase.ControlType.kPosition, 0, armFeedVolts);
        SmartDashboard.putNumber("feedforward volts", armFeedVolts);
        SmartDashboard.putNumber("pid volts", armMotorMaster.getBusVoltage() * armMotorMaster.getAppliedOutput() - armFeedVolts);
    }

    /**
     *
     * @param targetPos in radians
     */
    public void setArmTarget(double targetPos) {
        targetPos = getArmClampedGoal(targetPos);

        goalState.position = targetPos;
        goalState.velocity = 0;
    }

    public void resetGoal() {
        double armPos = getArmPos();
        setArmTarget(armPos);
    }

    public void driveMotor(Measure<Voltage> volts) {
        armMotorMaster.setVoltage(volts.in(Volts));

    }

    // #region Getters

    public double getArmPos() {
        return MathUtil.inputModulus(armMasterEncoder.getPosition(), ARM_DISCONT_RAD,
                ARM_DISCONT_RAD + 2 * Math.PI);
    }

    public double getArmVel() {
        return armMasterEncoder.getVelocity();
    }

    public TrapezoidProfile.State getCurrentArmState() {
        return new TrapezoidProfile.State(getArmPos(), getArmVel());
    }

    public TrapezoidProfile.State getCurrentArmGoal() {
        return goalState;
    }

    public boolean armAtSetpoint() {
        return Math.abs(getArmPos() - goalState.position) < POS_TOLERANCE_RAD &&
                Math.abs(getArmVel() - goalState.velocity) < VEL_TOLERANCE_RAD_P_SEC;
    }

    public double getArmClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONT_RAD, ARM_DISCONT_RAD + 2 * Math.PI),
                LOWER_ANGLE_LIMIT_RAD, UPPER_ANGLE_LIMIT_RAD);
    }

    public double getMaxAccelRad(){
        return armFeed.maxAchievableAcceleration(MAX_VOLTAGE, getArmPos(), getArmVel());
    }

    public double getMaxVelRad(){
        return armFeed.maxAchievableVelocity(MAX_VOLTAGE, getArmPos(), getArmVel());
    }
}
