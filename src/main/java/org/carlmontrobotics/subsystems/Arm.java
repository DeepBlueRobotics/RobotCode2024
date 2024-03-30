package org.carlmontrobotics.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.carlmontrobotics.Constants.Armc.*;

import org.carlmontrobotics.commands.TeleopArm;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// Arm angle is measured from horizontal on the intake side of the robot and bounded between -3π/2 and π/2
public class Arm extends SubsystemBase {
    private boolean callDrive = true;
    private final CANSparkMax armMotorMaster/* left */ = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT_MASTER,
            MotorConfig.NEO);
    private final CANSparkMax armMotorFollower/* right */ = MotorControllerFactory
            .createSparkMax(ARM_MOTOR_PORT_FOLLOWER, MotorConfig.NEO);
    private final SparkAbsoluteEncoder armMasterEncoder = armMotorMaster
            .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private static double kDt = 0.02;// what is this?

    // PID, feedforward, trap profile

    // rel offset = starting absolute offset
    private double armFeedVolts;// for SendableBuilder
    private final ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
    private final SparkPIDController armPIDMaster = armMotorMaster.getPIDController();
    private TrapezoidProfile.State setpoint = getCurrentArmState();

    private TrapezoidProfile armProfile;
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);

    private double lastMeasuredTime;
    private double lastArmPos;

    private boolean isArmEncoderConnected = false;

    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RadiansPerSecond.of(0));
    private final MutableMeasure<Angle> distance = mutable(Radians.of(0));

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
        armMasterEncoder.setInverted(ENCODER_INVERTED);

        armMasterEncoder.setZeroOffset(ENCODER_OFFSET_RAD);

        // ------------------------------------------------------------

        armMotorFollower.follow(armMotorMaster, MOTOR_INVERTED_FOLLOWER);

        armPIDMaster.setP(kP);
        // //SmartDashboard.putNumber("set KP", kP);
        // //armPIDMaster.setP(SmartDashboard.getNumber("set KP", kP));
        // SmartDashboard.putNumber("set KP", kP);
        // armPIDMaster.setP(SmartDashboard.getNumber("set KP", kP));
        // armPIDMaster.setI(kI);
        // SmartDashboard.putNumber("set kD", kD);
        // armPIDMaster.setD(SmartDashboard.getNumber("set kD", kD));
        // SmartDashboard.putNumber("set kG", kG);
        // // armPIDMaster.setD(SmartDashboard.getNumber("set kD", kD));
        armPIDMaster.setOutputRange(MIN_VOLTAGE / 12, MAX_VOLTAGE / 12);
        // // armPID.setTolerance(posToleranceRad, velToleranceRadPSec);

        armPIDMaster.setFeedbackDevice(armMasterEncoder);
        armPIDMaster.setPositionPIDWrappingEnabled(true);
        armPIDMaster.setPositionPIDWrappingMinInput(-Math.PI / 2);
        armPIDMaster.setPositionPIDWrappingMaxInput((3 * Math.PI) / 2);
        armPIDMaster.setIZone(IZONE_RAD);

        TRAP_CONSTRAINTS = new TrapezoidProfile.Constraints(
                Math.PI * .5,
                MAX_FF_ACCEL_RAD_P_S);
        // ^ worst case scenario
        // armFeed.maxAchievableVelocity(12, 0, MAX_FF_ACCEL_RAD_P_S)
        armProfile = new TrapezoidProfile(TRAP_CONSTRAINTS);

        SmartDashboard.putData("Arm", this);

        setpoint = getCurrentArmState();
        goalState = getCurrentArmState();
        setArmTarget(goalState.position);

        // sysid
        // sysid buttons on smartdashbaord; sysid tab name is arm sysid
        sysIdTab.add("quasistatic forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysIdTab.add("quasistatic backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysIdTab.add("dynamic forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysIdTab.add("dynamic backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putNumber("arm initial position", goalState.position);
        SmartDashboard.putNumber("set arm angle (rad)", 0);
        // sysid

        lastArmPos = getArmPos();
        lastMeasuredTime = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("ramp rate (s)", 2);
        SmartDashboard.putNumber("soft limit pos (rad)", SOFT_LIMIT_LOCATION_IN_RADIANS);
        armMotorMaster.setSmartCurrentLimit(80);
        armMotorFollower.setSmartCurrentLimit(80);

    }

    public void setBooleanDrive(boolean climb) {
        callDrive = climb;
    }

    @Override
    public void periodic() {

        // armMotorMaster.setSmartCurrentLimit(50);
        // armMotorFollower.setSmartCurrentLimit(50);
        if (DriverStation.isDisabled())
            resetGoal();

        // ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD =
        // SmartDashboard.getNumber("ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD",
        // ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);
        // armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL , MAX_FF_ACCEL
        // );

        // SmartDashboard.putNumber("KP", );
        // smart dahsboard stuff
        // SmartDashboard.putBoolean("ArmPIDAtSetpoint", armAtSetpoint());
        // SmartDashboard.putNumber("Arm Goal Pos (rad)", goalState.position);
        // SmartDashboard.putBoolean("ArmProfileFinished",
        // armProfile.isFinished(armProfileTimer.get()));
        // posToleranceRad = SmartDashboard.getNumber("Arm Tolerance Pos",
        // posToleranceRad);
        // velToleranceRadPSec= SmartDashboard.getNumber("Arm Tolerance Vel",
        // velToleranceRadPSec);

        // SmartDashboard.putNumber("InternalArmVelocity",
        // armMasterEncoder.getVelocity());
        // SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());

        // SmartDashboard.putNumber("ArmPos", getArmPos());
        double currTime = Timer.getFPGATimestamp();
        // SmartDashboard.putNumber("Current Time", currTime);
        // SmartDashboard.putNumber("Last Update (s)", lastMeasuredTime);
        // setArmTarget(SmartDashboard.getNumber("set arm angle (rad)", 0));

        // double currG = SmartDashboard.getNumber("set kG", kG);
        // double KG = kG;
        // if (currG != KG) {
        // armFeed = new ArmFeedforward(kS, SmartDashboard.getNumber("set kG", currG),
        // kV, kA);
        // KG = currG;
        // }

        SmartDashboard.putNumber("Master RPM", armMotorMaster.getEncoder().getVelocity());
        SmartDashboard.putNumber("Follower RPM", armMotorFollower.getEncoder().getVelocity());
        SmartDashboard.putNumber("Actual Master Arm Volts",
                armMotorMaster.getBusVoltage() * armMotorMaster.getAppliedOutput());
        SmartDashboard.putNumber("Actual Follower Arm Volts",
                armMotorFollower.getBusVoltage() * armMotorFollower.getAppliedOutput());

        // when the value is different
        
          double currentArmPos = getArmPos();
          if (currentArmPos != lastArmPos) {
          lastMeasuredTime = currTime;
          lastArmPos = currentArmPos;
          }
          isArmEncoderConnected = currTime - lastMeasuredTime <
          DISCONNECTED_ENCODER_TIMEOUT_SEC;
          
          if (isArmEncoderConnected) {
          if (callDrive) {
          driveArm();
          }
          } else {
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
            // kg * cos(arm angle) * arm_COM_length
        }
        armPIDMaster.setReference((setpoint.position), CANSparkBase.ControlType.kPosition, 0, armFeedVolts);

        SmartDashboard.putNumber("feedforward volts", armFeedVolts);
        SmartDashboard.putNumber("pid volts",
                armMotorMaster.getBusVoltage() * armMotorMaster.getAppliedOutput() - armFeedVolts);
    }

    public void stopArm() {
        armMotorMaster.set(0);

    }

    public void driveArm(double volts) {
        // armMotorMaster.set(0)
        armMotorMaster.setVoltage(volts); // STARTING WITH SLOWER SPEED FOR TESTING
    }

    public void setLimitsForClimbOn() {
        armPIDMaster.setOutputRange(-12, 12);
        armMotorMaster.setSoftLimit(SoftLimitDirection.kReverse,
                (float) SmartDashboard.getNumber("soft limit pos (rad)", SOFT_LIMIT_LOCATION_IN_RADIANS));
        armMotorMaster.setOpenLoopRampRate(SmartDashboard.getNumber("ramp rate (s)", 2));
        armMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);
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

    private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)),
            Volts.of(2), Seconds.of(10));

    public void logMotor(SysIdRoutineLog log) {
        log.motor("armMotorMaster")
                .voltage(voltage.mut_replace(
                        armMotorMaster.getBusVoltage() * armMotorMaster.getAppliedOutput(),
                        Volts))
                .angularVelocity(velocity.mut_replace(
                        armMasterEncoder.getVelocity(),
                        RadiansPerSecond))
                .angularPosition(distance.mut_replace(
                        armMasterEncoder.getPosition(),
                        Radians));
    }

    private final SysIdRoutine routine = new SysIdRoutine(
            defaultSysIdConfig,
            new SysIdRoutine.Mechanism(
                    this::driveMotor,
                    this::logMotor,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(new InstantCommand(() -> armMasterEncoder.setZeroOffset(0)),
                routine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(new InstantCommand(() -> armMasterEncoder.setZeroOffset(0)),
                routine.dynamic(direction));
    }

    // #region Getters

    public double getArmPos() {
        return MathUtil.inputModulus(armMasterEncoder.getPosition(), ARM_DISCONT_RAD,
                ARM_DISCONT_RAD + 2 * Math.PI);// armMasterEncoder.getPosition();//MathUtil.inputModulus(armMasterEncoder.getPosition(),
                                               // ARM_DISCONT_RAD,
        // ARM_DISCONT_RAD + 2 * Math.PI);
    }

    public double getArmVel() {
        return armMasterEncoder.getVelocity();
    }

    public void resetSoftLimit() {
        armPIDMaster.setOutputRange(MIN_VOLTAGE / -12, MAX_VOLTAGE / 12);
        armMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, false);
        armMotorMaster.setOpenLoopRampRate(0);
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

    public double getMaxAccelRad() {
        return armFeed.maxAchievableAcceleration(MAX_VOLTAGE, getArmPos(), getArmVel());
    }

    public double getMaxVelocity() {
        return TRAP_CONSTRAINTS.maxVelocity;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("armKp", () -> armPIDMaster.getP(), armPIDMaster::setP);
        builder.addDoubleProperty("armKd", () -> armPIDMaster.getD(), armPIDMaster::setD);
        builder.addDoubleProperty("Current Position", () -> getArmPos(), null);
        builder.addBooleanProperty("ArmPIDAtSetpoint", () -> armAtSetpoint(), null);
        builder.addDoubleProperty("Arm Goal Pos (rad)", () -> goalState.position, null);
        builder.addBooleanProperty("ArmEncoderConnected", () -> isArmEncoderConnected, null);
        builder.addDoubleProperty("feedforward volts", () -> armFeedVolts, null);
        builder.addDoubleProperty("pid volts",
                () -> armMotorMaster.getBusVoltage() * armMotorMaster.getAppliedOutput() - armFeedVolts, null);
        builder.addDoubleProperty("setpoint goal (rad)", () -> setpoint.position, null);
        builder.addDoubleProperty("setpoint velocity", () -> setpoint.velocity, null);

        builder.addDoubleProperty("arm initial position", () -> goalState.position, null);
        // builder.addDoubleProperty("set arm angle (rad)", () ->
        // armMasterEncoder.getPosition(), setArmTarget());
        builder.addBooleanProperty("ArmPIDAtSetpoint", () -> armAtSetpoint(), null);
        builder.addDoubleProperty("Arm Goal Pos (rad)", () -> goalState.position, null);
        builder.addDoubleProperty("InternalArmVelocity", () -> armMasterEncoder.getVelocity(), null);
        builder.addDoubleProperty("Soft limit Forward", () -> armMotorMaster.getSoftLimit(SoftLimitDirection.kForward),
                null);
        builder.addDoubleProperty("Soft limit Reverse", () -> armMotorMaster.getSoftLimit(SoftLimitDirection.kReverse),
                null);
    }

    public double getMaxVelRad() {
        return armFeed.maxAchievableVelocity(MAX_VOLTAGE, getArmPos(), getArmVel());
    }

    public void setDefaultCommand(TeleopArm teleopArm, Object object) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
    }
}
