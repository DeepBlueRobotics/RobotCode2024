// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.commands.ArmTeleop;

import static org.carlmontrobotics.Constants.Arm.*;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;



public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor1 = MotorControllerFactory.createSparkMax(LEFT_MOTOR_PORT,MotorConfig.NEO);
    //private final CANSparkMax armMotor2 = MotorControllerFactory.createSparkMax(Constants.Arm.RIGHT_MOTOR_PORT,MotorConfig.NEO);
    //there is only one arm motor
    //test

    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS, kV);
    private final SparkAbsoluteEncoder armEncoder = armMotor1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private final SparkPIDController armPID = armMotor1.getPIDController();
    public static TrapezoidProfile.State[] goalState = { 
      new TrapezoidProfile.State(intakeAngle, 0), 
      new TrapezoidProfile.State(ampAngle, 0),  
      new TrapezoidProfile.State(placeholderSpeakerAngle1, 0),
      new TrapezoidProfile.State(placeholderSpeakerAngle2, 0),
      new TrapezoidProfile.State(placeholderSpeakerAngle3, 0),
      new TrapezoidProfile.State(climberUpAngle, 0),
      new TrapezoidProfile.State(climberDownAngle, 0),
    };
    
    TrapezoidProfile profile = new TrapezoidProfile(Constants.Arm.trapConstraints);
    
    public Arm() {
      armPID.setP(kP);
      armPID.setI(kI);
      armPID.setD(kD);
      armPID.setFF(kV,0);
      armPID.setIZone(IZONE); //<-- zero for now
      SmartDashboard.putNumber("kp", kP);
      SmartDashboard.putNumber("kd", kD);
      SmartDashboard.putNumber("ki", kI);
      SmartDashboard.putNumber("Shooter RPS", 0);

    }

			//arm 
      /*
       have 3 set positions
       Speaker position 
       Amp position
        ground position
       These set positions would also be conrtolled by buttons

       There will also be a manual control for the arm using the right joystick
      */
	

    public TrapezoidProfile.State calculateSetPoint(double goalSeconds, TrapezoidProfile.State currentPoint, int goalStateIndex) {
      return profile.calculate(goalSeconds, currentPoint, goalState[goalStateIndex]);
    }

    public TrapezoidProfile.State calculateCustomSetPoint(double goalSeconds, TrapezoidProfile.State currentPoint, TrapezoidProfile.State goalState) {
      return profile.calculate(goalSeconds, currentPoint, goalState);
      
    }

    
    //public void maxacceleration(){
      //armFeed.maxAchievableAcceleration(MAX_VOLTAGE, armEncoder.getVelocity()); 
    //}

    double currentVelocity = armEncoder.getVelocity();

    public double maxacceleration(){
      double maxAccel = armFeed.maxAchievableAcceleration(MAX_VOLTAGE, currentVelocity); 
      return maxAccel;
    }

    //public void maxvelocity(){
      //armFeed.maxAchievableVelocity(MAX_VOLTAGE, );
    //}

    public double calculateTrapTime(double goalAngle, double currentAngle){
      double distToCover = goalAngle - currentAngle;
      double maxAccel = maxacceleration();

      double timeOfAccelTriangle = MAX_FF_VEL / maxAccel;
      double distCovrdTriangle = maxAccel / 2 * timeOfAccelTriangle;

      double distLeft = distToCover - (2 * distCovrdTriangle);
      double timeOfRect = distLeft / MAX_FF_VEL;

      double answer = timeOfAccelTriangle*2 + timeOfRect;
      return answer;
    }
    

    // public void driveArm(double timeToTarget, TrapezoidProfile.State goalState) {
    //   TrapezoidProfile.State setPoint = profile.calculate(timeToTarget, getCurrentArmState(), goalState);
    //   double armFeedVolts = armFeed.calculate(goalState.velocity, 0);
    //   armPID.setReference(setPoint.position, CANSparkMax.ControlType.kPosition, 0, armFeedVolts);

    // }
    

    public void COMBINE_PID_FF_TRAPEZOID(TrapezoidProfile.State setPoint) {
      // feed forward still needs the math part
      double armFeedVolts = armFeed.calculate(setPoint.velocity, 0);
      armPID.setReference(setPoint.position, CANSparkMax.ControlType.kPosition,0, armFeedVolts);
    }

    public double getArmPos() {
      return MathUtil.inputModulus(armEncoder.getPosition(), ARM_DISCONT_RAD,
              ARM_DISCONT_RAD + 2 * Math.PI);
    } 

    public void cancelArmCommand() {
      Command currentArmCommand = getCurrentCommand();
      currentArmCommand.cancel();
    }

    public Translation2d getCoM() {
      // the constants are placeholders
      Translation2d comOfArm = new Translation2d(COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .times(ARM_MASS_KG);
      return comOfArm;
    }

    public double getArmVel() {
      return armEncoder.getVelocity();
    }

    public double getArmClampedGoal(double goal) {
      //Find the limits of the arm. Used to move it and ensure that the arm does not move past the amount
      return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONT_RAD, ARM_DISCONT_RAD + 2 * Math.PI), LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT);
    }

    public TrapezoidProfile.State getCurrentArmState() {
      return new TrapezoidProfile.State(getArmPos(), getArmVel());
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


    @Override
    public void periodic() {

      autoCancelArmCommand();



      // kP = SmartDashboard.getNumber("kP", kP);
      // kD = SmartDashboard.getNumber("kD", kD);
      // kI = SmartDashboard.getNumber("kI", kI);

      if (armPID.getP() != kP) {
          armPID.setP(kP);
      }
      if (armPID.getD() != kD) {
          armPID.setD(kD);
      }
      if (armPID.getI() != kI) {
          armPID.setI(kI);
      }

		}
}
