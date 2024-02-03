// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TimeOfFlight distSensor = new TimeOfFlight(10);
  private double DSdepth = 9.97;
  private double DSdetectdistance = 13;
  private CANSparkMax motor = MotorControllerFactory.createSparkMax(9, MotorConfig.NEO);
  public Intake() {
    SmartDashboard.putNumber("motor voltage", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Detect Distance", getGamePieceDistanceIn());
    SmartDashboard.putBoolean("Has GUM", hasGamePiece());
    if (hasGamePiece()) motor.set(0);
    else motor.set(SmartDashboard.getNumber("motor voltage", 0));
  }
  public boolean hasGamePiece() {
    // return false;
    return getGamePieceDistanceIn() < DSdetectdistance;
  }

  public double getGamePieceDistanceIn() {
    return Units.metersToInches((distSensor.getRange() - DSdepth) / 1000 /* Convert mm to m */);
  }
}
