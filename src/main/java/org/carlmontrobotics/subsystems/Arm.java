// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import static org.carlmontrobotics.Constants.Arm.*;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
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
    //there is only one arm motor. 
    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS, kV);
    private final SparkAbsoluteEncoder armEncoder = armMotor1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private final PIDController armPID = new PIDController(kP, kI, kD);
    public static TrapezoidProfile.State[] goalState = { new TrapezoidProfile.State(-Math.PI / 2, 0), new TrapezoidProfile.State(Math.toRadians(43), 0) };
   
    TrapezoidProfile.Constraints constraints =new TrapezoidProfile.Constraints(kMaxV, kMaxA);

    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    
  
			//arm 
      /*
       have 3 set positions
       Speaker position 
       Amp position
        ground position
       These set positions would also be conrtolled by buttons

       There will also be a manual control for the arm using the right joystick
      */
    }
	
    public void setArmGoal(double targetPosition, double targetVelocity) {
      //Sets arm to the optimal angle for amp, speaker and Ground intake | used to score in amp
      //these values are in constants
      //pass in where scorign and use switch statement to alternate between each angle needed
      targetPosition = getArmClampedGoal(targetPosition);

      
    }

    public double getArmPos() {
      return MathUtil.inputModulus(armEncoder.getPosition(), Constants.Arm.ARM_DICONT_RAD,
              Constants.Arm.ARM_DICONT_RAD + 2 * Math.PI);
  }

    public double getKg() {
      return 17; //This is a placeholder number
    }

    public Translation2d getCoM() {
      // the constants are placeholders
      Translation2d comOfArm = new Translation2d(Constants.Arm.COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .times(Constants.Arm.ARM_MASS_KG);
      return comOfArm;
    }

    public double getArmVel() {
      return armEncoder.getVelocity();
  }

    public void driveArm(TrapezoidProfile.State state) {

      /*
      ignore this math its wrong as it includes wrist 
        double kgv = getKg();
        double armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(state.velocity, 0);
        double armPIDVolts = armPID.calculate(getArmPos(), state.position);
        if ((getArmPos() > Constants.Arm.UPPER_ANGLE_LIMIT && state.velocity > 0) || 
            (getArmPos() < Constants.Arm.LOWER_ANGLE_LIMIT && state.velocity < 0)) {
            armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
    }
    double volts = armFeedVolts + armPIDVolts;
    armMotor1.setVoltage(volts);
  
  */
    }
    public double getArmClampedGoal(double goal) {
      //Find the limits of the arm. Used to move it and ensure that the arm does not move past the amount
      return MathUtil.clamp(MathUtil.inputModulus(goal, Constants.Arm.ARM_DICONT_RAD, Constants.Arm.ARM_DICONT_RAD + 2 * Math.PI), Constants.Arm.LOWER_ANGLE_LIMIT, Constants.Arm.UPPER_ANGLE_LIMIT);
    }

    public TrapezoidProfile.State getCurrentArmState() {
      return new TrapezoidProfile.State(getArmPos(), getArmVel());
    }

    @Override
    public void periodic() {
      
		}
}
