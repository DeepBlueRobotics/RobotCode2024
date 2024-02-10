// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {

 
 
  /** Creates a new Intake. */

  /* 
  private TimeOfFlight distSensor = new TimeOfFlight(12);
  private double DSdepth = 9.97;
  private double DSdetectdistance = 13;
  private CANSparkMax motor1 = MotorControllerFactory.createSparkMax(7, MotorConfig.NEO);
  private RelativeEncoder motorEncoder = motor1.getEncoder();
  private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
  private final MutableMeasure<Velocity<Angle>> angleVelocity = mutable(RotationsPerSecond.of(0));
  
 // private final SysIdRoutine SysIdRoutine = 
   /*  new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
    );
*/
//  public Intake() {
 //   SmartDashboard.putNumber("motor voltage", 0);
 // }

/*   public void voltageDrive(Measure<Voltage> volts) {
    motor1.setVoltage(volts.in(Volts));
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return SysIdRoutine.quasistatic(direction);
} 
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return SysIdRoutine.dynamic(direction);
}
  public void logMotors(SysIdRoutineLog log) {
      log.motor("intake-motor")
        .voltage(voltage.mut_replace(motor1.getBusVoltage(),
        Volts
      )).angularVelocity(angleVelocity.mut_replace(motorEncoder.getVelocity(),
      RPM));

  }
  public void loadPerferences() {
  
  }
  */

 /*  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Detect Distance", getGamePieceDistanceIn());
    SmartDashboard.putBoolean("Has GUM", hasGamePiece());
    
 //   if (hasGamePiece()) 
    {

 //     motor1.set(0);    
    }
 //   else motor1.set(SmartDashboard.getNumber("motor voltage", 0));
    
/*   
  }

  public boolean hasGamePiece() {
    // return false;
    return getGamePieceDistanceIn() < DSdetectdistance;
  }

  public double getGamePieceDistanceIn() {
    return Units.metersToInches((distSensor.getRange() - DSdepth) / 1000 /* Convert mm to m */
  //}

//}
