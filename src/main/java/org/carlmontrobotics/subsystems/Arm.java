package org.carlmontrobotics.subsystems;
import static org.carlmontrobotics.Constants.Arm.*;

import org.carlmontrobotics.commands.ArmTeleop;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

// Arm angle is measured from horizontal on the intake side of the robot and bounded between -3π/2 and π/2
// Wrist angle is measured relative to the arm with 0 being parallel to the arm and bounded between -π and π (Center of Mass of Roller)
public class Arm extends SubsystemBase {
  ///GOALS
  /*
   * public static final double intakeAngle = Math.toRadians(0);
		public static final double ampAngle = Math.toRadians(103);
		public static final double placeholderSpeakerAngle1 = Math.toRadians(24);
		public static final double placeholderSpeakerAngle2 = Math.toRadians(24);
		public static final double placeholderSpeakerAngle3 = Math.toRadians(24);
		public static final double climberUpAngle = Math.toRadians(24);
		public static final double climberDownAngle = Math.toRadians(24);
   */
    // a boolean meant to tell if the arm is in a forbidden posistion AKA FORBIDDEN FLAG
    
    private final CANSparkMax masterMotor = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT_1, MotorConfig.NEO);
    private final CANSparkMax followerMotor = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT_2, MotorConfig.NEO);
    private final RelativeEncoder armEncoder = masterMotor.getEncoder();


    private static double kDt = 0.02;
   
    //PID, feedforward, trap profile
    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS, kV, kA);
    private final SparkPIDController armPID1 = masterMotor.getPIDController();
    private final SparkPIDController armPID2 = followerMotor.getPIDController();
    private TrapezoidProfile armProfile = new TrapezoidProfile(trapConstraints);
    private Timer armProfileTimer = new Timer();
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(0,0);//TODO: update pos later

    // rad, rad/s
    //public static TrapezoidProfile.State[] goalState = { new TrapezoidProfile.State(-Math.PI / 2, 0), new TrapezoidProfile.State(0, 0) };

    public Arm() {
      // weird math stuff
        masterMotor.setInverted(MOTOR_INVERTED);
        masterMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.setInverted(MOTOR_INVERTED);
        followerMotor.setIdleMode(IdleMode.kBrake);
        armEncoder.setPositionConversionFactor(ROTATION_TO_RAD);
        armEncoder.setVelocityConversionFactor(ROTATION_TO_RAD);
        armEncoder.setInverted(ENCODER_INVERTED);

        followerMotor.follow(masterMotor);
     
        //armEncoder1.setZeroOffset(offsetRad);
      
        //armPID.setTolerance(posToleranceRad, velToleranceRadPSec);

        SmartDashboard.putData("Arm", this);

        //armProfileTimer.start(); <-- don't neeed timer anymore

        setArmTarget(goalState.position);

        // SmartDashboard.putNumber("Arm Max Vel", MAX_FF_VEL );
        SmartDashboard.putNumber("ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD", ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);
        SmartDashboard.putNumber("Arm Tolerance Pos", POS_TOLERANCE_RAD);
        SmartDashboard.putNumber("Arm Tolerance Vel", VEL_TOLERANCE_RAD_P_SEC);
    }

    @Override
    public void periodic() {

        if(DriverStation.isDisabled()) resetGoal();

        //ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = SmartDashboard.getNumber("ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD", ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);
        // armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL , MAX_FF_ACCEL );
        armPID1.setP(kP);
        armPID1.setI(kI);
        armPID1.setD(kD);
        armPID2.setP(kP);
        armPID2.setI(kI);
        armPID2.setD(kD);

        //smart dahsboard stuff
        //SmartDashboard.putBoolean("ArmPIDAtSetpoint", armPID1.atSetpoint());
        SmartDashboard.putBoolean("ArmProfileFinished", armProfile.isFinished(armProfileTimer.get()));
        //posToleranceRad = SmartDashboard.getNumber("Arm Tolerance Pos", posToleranceRad);
        //velToleranceRadPSec= SmartDashboard.getNumber("Arm Tolerance Vel", velToleranceRadPSec);

       // SmartDashboard.putNumber("MaxHoldingTorque", maxHoldingTorqueNM());
        //SmartDashboard.putNumber("V_PER_NM", getV_PER_NM());
       // SmartDashboard.putNumber("COMDistance", getCoM().getNorm());
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
    
      armPID1.setReference(setPoint.position, CANSparkBase.ControlType.kVelocity, 0, armFeedVolts);
    }

    public void setArmTarget(double targetPos) {
        targetPos = getArmClampedGoal(targetPos);

        armProfile = new TrapezoidProfile(trapConstraints);
        armProfileTimer.reset();

        goalState.position = targetPos;
        goalState.velocity = 0;
    }

    

    public void resetGoal() {
        double armPos = getArmPos();
      
        armProfile = new TrapezoidProfile(trapConstraints);

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

    

    


   
}