package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Arm.*;

import org.carlmontrobotics.commands.ArmTeleop;
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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
// Wrist angle is measured relative to the arm with 0 being parallel to the arm and bounded between -π and π (Center of Mass of Roller)
public class Arm extends SubsystemBase {

    private final CANSparkMax armMotorMaster/* left */ = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT_MASTER,
            MotorConfig.NEO);
    private final CANSparkMax armMotorFollower/* right */ = MotorControllerFactory
            .createSparkMax(ARM_MOTOR_PORT_FOLLOWER, MotorConfig.NEO);
    private final SparkAbsoluteEncoder armMasterEncoder = armMotorMaster
            .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
    private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));

    private static double kDt = 0.02;

    // PID, feedforward, trap profile

    // rel offset = starting absolute offset
    private final ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
    private final SparkPIDController armPIDMaster = armMotorMaster.getPIDController();
    private static TrapezoidProfile.State setpoint;

    private TrapezoidProfile armProfile = new TrapezoidProfile(TRAP_CONSTRAINTS);
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);// TODO: update pos later

    // rad, rad/s
    // public static TrapezoidProfile.State[] goalState = { new
    // TrapezoidProfile.State(-Math.PI / 2, 0), new TrapezoidProfile.State(0, 0) };


    private ShuffleboardTab sysIdTab = Shuffleboard.getTab("arm SysID");

    public Arm() {
        // weird math stuff
        armMotorMaster.setInverted(MOTOR_INVERTED_MASTER);
        armMotorMaster.setIdleMode(IdleMode.kBrake);
        armMotorFollower.setInverted(MOTOR_INVERTED_FOLLOWER);
        armMotorFollower.setIdleMode(IdleMode.kBrake);
        // Comment out when running sysid
        armMasterEncoder.setPositionConversionFactor(ROTATION_TO_RAD);
        armMasterEncoder.setVelocityConversionFactor(ROTATION_TO_RAD);
        // ------------------------------------------------------------
        armMasterEncoder.setInverted(ENCODER_INVERTED);

        armMotorFollower.follow(armMotorMaster, MOTOR_INVERTED_FOLLOWER);
        armPIDMaster.setP(kP);
        armPIDMaster.setI(kI);
        armPIDMaster.setD(kD);

        // armPID.setTolerance(posToleranceRad, velToleranceRadPSec);

        armPIDMaster.setFeedbackDevice(armMasterEncoder);
        armPIDMaster.setPositionPIDWrappingEnabled(true);
        armPIDMaster.setPositionPIDWrappingMinInput(LOWER_ANGLE_LIMIT_RAD);
        armPIDMaster.setPositionPIDWrappingMaxInput(UPPER_ANGLE_LIMIT_RAD);
        armPIDMaster.setIZone(IZONE_RAD)

        SmartDashboard.putData("Arm", this);

        setpoint = getCurrentArmState();
        goalState = getCurrentArmState();
        setArmTarget(goalState.position);

        //sysid buttons on smartdashbaord; sysid tab name is arm sysid
        sysIdTab.add("quasistatic forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysIdTab.add("quasistatic backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysIdTab.add("dynamic forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysIdTab.add("dynamic backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));
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

        // smart dahsboard stuff
        // SmartDashboard.putBoolean("ArmPIDAtSetpoint", armPID1.atSetpoint());
        // SmartDashboard.putBoolean("ArmProfileFinished",
        // armProfile.isFinished(armProfileTimer.get()));
        // posToleranceRad = SmartDashboard.getNumber("Arm Tolerance Pos",
        // posToleranceRad);
        // velToleranceRadPSec= SmartDashboard.getNumber("Arm Tolerance Vel",
        // velToleranceRadPSec);

        // SmartDashboard.putNumber("MaxHoldingTorque", maxHoldingTorqueNM());
        // SmartDashboard.putNumber("V_PER_NM", getV_PER_NM());
        // SmartDashboard.putNumber("COMDistance", getCoM().getNorm());
        SmartDashboard.putNumber("InternalArmVelocity", armMasterEncoder.getVelocity());
        // SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());

        // SmartDashboard.putNumber("ArmPos", getArmPos());

        autoCancelArmCommand();
        driveArm();
    }

    public void autoCancelArmCommand() {
        if (!(getDefaultCommand() instanceof ArmTeleop) || DriverStation.isAutonomous())
            return;

        double requestedSpeeds = ((ArmTeleop) getDefaultCommand()).getRequestedSpeeds();

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
        double armFeedVolts = armFeed.calculate(getArmPos(), setpoint.velocity);
        if ((getArmPos() < LOWER_ANGLE_LIMIT_RAD && getCurrentArmGoal().velocity > 0)
                || (getArmPos() > UPPER_ANGLE_LIMIT_RAD && getCurrentArmGoal().velocity > 0)) {
            armFeedVolts = armFeed.calculate(getCurrentArmGoal().position, 0);
        }
        armPIDMaster.setReference(setpoint.position, CANSparkBase.ControlType.kPosition, 0, armFeedVolts);
    }

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
    private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(2), Seconds.of(10));

    public void logMotor(SysIdRoutineLog log) {
        log.motor("armMotorMaster")
                .voltage(voltage.mut_replace(
                        armMotorMaster.getBusVoltage() * armMotorMaster.getAppliedOutput(),
                        Volts))
                .angularVelocity(velocity.mut_replace(
                        armMasterEncoder.getVelocity() / 60,
                        RotationsPerSecond))
                .angularPosition(distance.mut_replace(
                        armMasterEncoder.getPosition(),
                        Rotations));
    }

    private final SysIdRoutine routine = new SysIdRoutine(
            defaultSysIdConfig,
            new SysIdRoutine.Mechanism(
                    this::driveMotor,
                    this::logMotor,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
    // #endregion

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
}
