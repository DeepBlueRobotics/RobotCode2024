

import static org.carlmontrobotics.Constants.Arm.*;

import org.carlmontrobotics.commands.ArmTeleop;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

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
    // a boolean meant to tell if the arm is in a forbidden posistion AKA FORBIDDEN FLAG
    private static boolean forbFlag;
    private final CANSparkMax armMotor1 = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT_1, MotorConfig.NEO);
    private final CANSparkMax armMotor2 = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT_2, MotorConfig.NEO)
    private final RelativeEncoder armEncoder = armMotor1.getEncoder();
   

    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS, kV, kA);

    private final PIDController armPID = new PIDController(kP, kI, kD);

    private TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);
    private Timer armProfileTimer = new Timer();

    // rad, rad/s
    //public static TrapezoidProfile.State[] goalState = { new TrapezoidProfile.State(-Math.PI / 2, 0), new TrapezoidProfile.State(0, 0) };

    public Arm() {
        armMotor1.setInverted(motorInverted);
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setInverted(motorInverted);
        armMotor2.setIdleMode(IdleMode.kBrake);


        armEncoder.setPositionConversionFactor(rotationToRad);
        armEncoder.setVelocityConversionFactor(rotationToRad);
        armEncoder.setInverted(encoderInverted);
     
        //armEncoder1.setZeroOffset(offsetRad);
      
        armPID.setTolerance(posToleranceRad, velToleranceRadPSec);

        SmartDashboard.putData("Arm", this);

        armProfileTimer.start();

        setArmTarget(goalState.position, 0);

        // SmartDashboard.putNumber("Arm Max Vel", MAX_FF_VEL );
        // SmartDashboard.putNumber("Wrist Max Vel", MAX_FF_VEL[WRIST]);
        SmartDashboard.putNumber("ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD", ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);
        SmartDashboard.putNumber("Arm Tolerance Pos", posToleranceRad);
        SmartDashboard.putNumber("Arm Tolerance Vel", velToleranceRadPSec);
    }

    @Override
    public void periodic() {

        if(DriverStation.isDisabled()) resetGoal();

        ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = SmartDashboard.getNumber("ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD", ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);
        // wristConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL[WRIST], MAX_FF_ACCEL[WRIST]);
        // armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL , MAX_FF_ACCEL );
        armPID.setP(kP);
        armPID.setI(kI);
        armPID.setD(kD);
        SmartDashboard.putBoolean("ArmPIDAtSetpoint", armPID.atSetpoint());
        SmartDashboard.putBoolean("ArmProfileFinished", armProfile.isFinished(armProfileTimer.get()));
        posToleranceRad = SmartDashboard.getNumber("Arm Tolerance Pos", posToleranceRad);
        velToleranceRadPSec= SmartDashboard.getNumber("Arm Tolerance Vel", velToleranceRadPSec);
       

        SmartDashboard.putNumber("MaxHoldingTorque", maxHoldingTorqueNM());
        SmartDashboard.putNumber("V_PER_NM", getV_PER_NM());
        SmartDashboard.putNumber("COMDistance", getCoM().getNorm());
        SmartDashboard.putNumber("InternalArmVelocity", armRelEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());

        SmartDashboard.putNumber("ArmPos", getArmPos());

        driveArm(armProfile.calculate(armProfileTimer.get()));

        autoCancelArmCommand();
    }

    public void autoCancelArmCommand() {
        if(!(getDefaultCommand() instanceof ArmTeleop) || DriverStation.isAutonomous()) return;

        double[] requestedSpeeds = ((ArmTeleop) getDefaultCommand()).getRequestedSpeeds();

        if(requestedSpeeds[0] != 0 || requestedSpeeds[1] != 0) {
            Command currentArmCommand = getCurrentCommand();
            if(currentArmCommand != getDefaultCommand() && currentArmCommand != null) {
                currentArmCommand.cancel();
            }
        }
    }

    //#region Drive Methods

    private void driveArm(TrapezoidProfile.State state) {
       TrapezoidProfile.State setPoint = armProfile.calculate(timeToTarget, getCurrentArmState(), goalState);
        double armFeedVolts = armFeed.calculate(goalState.velocity, 0);
        armPID.setReference(setPoint.position, CANSparkMax.ControlType.kPosition, 0, armFeedVolts);
    }


    public void setArmTarget(double targetPos) {
        targetPos = getArmClampedGoal(targetPos);

     

        armProfile = new TrapezoidProfile(armConstraints, new TrapezoidProfile.State(targetPos, 0), armProfile.calculate(armProfileTimer.get()));
        armProfileTimer.reset();

        goalState.position = targetPos;
        goalState.velocity = 0;
    }

    

    public void resetGoal() {
        double armPos = getArmPos();
      
        armProfile = new TrapezoidProfile(armPos, 0);

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
        return goalState ;
    }

   
    public boolean armAtSetpoint() {
        return armPID.atSetpoint() && armProfile.isFinished(armProfileTimer.get());
    }

    

    //#endregion

    //#region Util Methods

    public double getArmClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONTINUITY_RAD, ARM_DISCONTINUITY_RAD + 2 * Math.PI), ARM_LOWER_LIMIT_RAD, ARM_UPPER_LIMIT_RAD);
    }

    public double getWristClampedGoal(double goal) {
    }

    public Translation2d getCoM() {
        Translation2d comOfArm = new Translation2d(COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .times(ARM_MASS_KG);
        
        return comOfArm.plus(comOfArm);
        //this math is prob wront
    }

    /*public double maxHoldingTorqueNM() {
        return (ARM_MASS_KG + ROLLER_MASS_KG) * g * getCoM().getNorm();
    }
    */

   /*  public static double getV_PER_NM() {
        double kg = kG ;
        double phi = 2.638;
        double Ma = ARM_MASS_KG;
        double Mr = ROLLER_MASS_KG;
        double Ra = ARM_LENGTH_METERS;
        double Rr = COM_ROLLER_LENGTH_METERS;
        double PaRa = COM_ARM_LENGTH_METERS;
        double g = 9.80;

        double c = (kg) / (g * Math.sqrt(Math.pow(Ma * PaRa + Mr * Ra, 2) + Math.pow(Mr * Rr, 2) + 2 * (Ma * PaRa + Mr * Ra) * (Mr * Rr) * Math.cos(phi)));
        return c;
    }*/

    /*public double getKg() {
        return getV_PER_NM() * maxHoldingTorqueNM();
    }*/

   

    
   
   /*  public boolean getForbFlag()
    {
        boolean output = forbFlag;
        forbFlag = false;//default: if it wasn't set to true, it's false
        return output;
    }*/
   

    

   
    //#endregion

    //#region SmartDashboard Methods

    @Override


    // In the scenario that initSendable method does not work like last time
   
   
    

   

    //#endregion

}