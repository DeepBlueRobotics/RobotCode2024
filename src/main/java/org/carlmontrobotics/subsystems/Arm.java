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
    private final SparkAbsoluteEncoder armEncoder = masterArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private static double kDt = 0.02;
   
    //PID, feedforward, trap profile
    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS, kV, kA);
    private final SparkPIDController armPID = masterArmMotor.getPIDController();

    private TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);
    
    //TODO: put in correct initial position
    // rad, rad/s
    private static TrapezoidProfile.State goalState = new TrapezoidProfile.State(0,0);


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

        setArmTarget(goalState.position);
    }

    @Override
    public void periodic() {

        if(DriverStation.isDisabled()) resetGoal();

        //smart dahsboard stuff
        //SmartDashboard.putBoolean("ArmPIDAtSetpoint", armPID1.atSetpoint());
        SmartDashboard.putBoolean("ArmProfileFinished", armProfile.isFinished(armProfileTimer.get()));
        //posToleranceRad = SmartDashboard.getNumber("Arm Tolerance Pos", posToleranceRad);
        //velToleranceRadPSec= SmartDashboard.getNumber("Arm Tolerance Vel", velToleranceRadPSec);

       // SmartDashboard.putNumber("MaxHoldingTorque", maxHoldingTorqueNM());
        //SmartDashboard.putNumber("V_PER_NM", getV_PER_NM());
        SmartDashboard.putNumber("COMDistance", getCoM().getNorm());
        SmartDashboard.putNumber("InternalArmVelocity", armEncoder.getVelocity());
        //SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());

       // SmartDashboard.putNumber("ArmPos", getArmPos());

        //driveArm(armProfile.calculate(armProfileTimer.get()));

        autoCancelArmCommand();
    }

  public void autoCancelArmCommand() {
        if(!(getDefaultCommand() instanceof ArmTeleop) || DriverStation.isAutonomous()) return;

        double requestedSpeeds = ((ArmTeleop) getDefaultCommand()).getRequestedSpeeds();

        if(requestedSpeeds != 0) {
            Command currentArmCommand = getCurrentCommand();
            if(currentArmCommand != getDefaultCommand() && currentArmCommand != null) {
                currentArmCommand.cancel();
            }
        }
    }

    //#region Drive Methods
    public void driveArm(double goalAngle){
      TrapezoidProfile.State goalState = new TrapezoidProfile.State(goalAngle, 0);
      TrapezoidProfile.State setPoint = armProfile.calculate(kDt, getCurrentArmState(), goalState);
      double armFeedVolts = armFeed.calculate(goalState.velocity, 0);
    
      armPID.setReference(setPoint.position, CANSparkBase.ControlType.kVelocity, 0, armFeedVolts);
    }

    public void setArmTarget(double targetPos) {
        targetPos = getArmClampedGoal(targetPos);

        armProfile = new TrapezoidProfile(armConstraints);
        armProfileTimer.reset();

        goalState.position = targetPos;
        goalState.velocity = 0;
    }

    

    public void resetGoal() {
        double armPos = getArmPos();
      
        armProfile = new TrapezoidProfile(armConstraints);

    }

    //#endregion

    //#region Getters

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

   
    public boolean armAtSetpoint() {
        return armProfile.isFinished(armProfileTimer.get());
    }



    public double getArmClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONTINUITY_RAD, ARM_DISCONTINUITY_RAD + 2 * Math.PI), ARM_LOWER_LIMIT_RAD, ARM_UPPER_LIMIT_RAD);
    }

    

    public Translation2d getCoM() {
        Translation2d comOfArm = new Translation2d(COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .times(ARM_MASS_KG);
        
        return comOfArm.plus(comOfArm);
        //this math is prob wront
    }

   
}