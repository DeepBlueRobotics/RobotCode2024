package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Arm.*;

import org.carlmontrobotics.commands.ArmTeleop;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: FIGURE OUT ANGLES
// Arm angle is measured from horizontal on the intake side of the robot and bounded between __ and __
public class Arm extends SubsystemBase {

    private final CANSparkMax masterArmMotor = MotorControllerFactory.createSparkMax(MASTER_ARM_MOTOR, MotorConfig.NEO);
    private final CANSparkMax followArmMotor = MotorControllerFactory.createSparkMax(FOLLOW_ARM_MOTOR, MotorConfig.NEO);
    private final SparkAbsoluteEncoder armEncoder = masterArmMotor
            .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private static double kDt = 0.02;

    // PID, feedforward, trap profile
    private final ArmFeedforward armFeed = new ArmFeedforward(kG, kS, kV, kA);
    private final SparkPIDController armPID = masterArmMotor.getPIDController();

    private TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

    // TODO: put in correct initial position
    // rad, rad/s
    private static TrapezoidProfile.State goalState;
    private static TrapezoidProfile.State setpointState;

    public Arm() {
        masterArmMotor.setInverted(motorInverted[MASTER]);
        masterArmMotor.setIdleMode(IdleMode.kBrake);
        followArmMotor.setInverted(motorInverted[FOLLOWER]);
        followArmMotor.setIdleMode(IdleMode.kBrake);
        armEncoder.setPositionConversionFactor(rotationToRad);
        armEncoder.setVelocityConversionFactor(rotationToRad);
        armEncoder.setInverted(encoderInverted);
        followArmMotor.follow(masterArmMotor);

        armEncoder.setZeroOffset(ENCODER_OFFSET);
        armPID.setFeedbackDevice(armEncoder);
        armPID.setPositionPIDWrappingEnabled(true);
        armPID.setPositionPIDWrappingMinInput(ARM_LOWER_LIMIT_RAD);
        armPID.setPositionPIDWrappingMaxInput(ARM_UPPER_LIMIT_RAD);

        armPID.setP(kP);
        armPID.setI(kI);
        armPID.setD(kD);

        SmartDashboard.putData("Arm", this);

        setpointState = getCurrentArmState();
        goalState = getCurrentArmState();

        setArmTarget(goalState.position);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled())
            resetGoal();
        driveArm();
        autoCancelArmCommand();
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

    // run in periodic, code that drives arm to desired goal
    // uses trapezoid profiles which supplies goal states to pid controller
    // feedforward controller is used to supply additional voltage to keep it
    // at its current position
    // YOU DO NOT HAVE TO WORRY ABOUT HOW THE ARM DRIVES OUTSIDE OF SUBSYSTEM
    public void driveArm() {
        setpointState = armProfile.calculate(kDt, setpointState, goalState);
        var currentPosition = getCurrentArmState();
        double armFeedVolts = armFeed.calculate(currentPosition.position, currentPosition.velocity);

        // code to stop arm from moving past certain bounds
        if ((getArmPos() > ARM_UPPER_LIMIT_RAD && currentPosition.velocity > 0) ||
                (getArmPos() < ARM_LOWER_LIMIT_RAD && currentPosition.velocity < 0)) {
            armFeedVolts = armFeed.calculate(currentPosition.position, 0);
        }
        armPID.setReference(setpointState.position, CANSparkBase.ControlType.kPosition, 0, armFeedVolts);
    }

    // sets target arm position
    // automatically clamps target positions such that they do not exceed lower and
    // upper limit
    public void setArmTarget(double targetPos) {
        targetPos = getArmClampedGoal(targetPos);
        goalState.position = targetPos;
        goalState.velocity = 0;
    }

    public void resetGoal() {
        double armPos = getArmPos();
        setArmTarget(armPos);
    }

    // #endregion

    // #region Getters

    public double getArmPos() {
        return MathUtil.inputModulus(armEncoder.getPosition(), ARM_DISCONTINUITY_RAD,
                ARM_DISCONTINUITY_RAD + 2 * Math.PI);
    }

    public double getArmVel() {
        return armEncoder.getVelocity();
    }

    public TrapezoidProfile.State getCurrentArmState() {
        return new TrapezoidProfile.State(getArmPos(), getArmVel());
    }

    public TrapezoidProfile.State getCurrentArmGoal() {
        return goalState;
    }

    public boolean armAtGoal() {
        return Math.abs(getArmPos() - goalState.position) < posToleranceRad &&
                Math.abs(getArmVel() - goalState.velocity) < velToleranceRadPSec;
    }

    public double getArmClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONTINUITY_RAD, ARM_DISCONTINUITY_RAD + 2 * Math.PI),
                ARM_LOWER_LIMIT_RAD, ARM_UPPER_LIMIT_RAD);
    }
}