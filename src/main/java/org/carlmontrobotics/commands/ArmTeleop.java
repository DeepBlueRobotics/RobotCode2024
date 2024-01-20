// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;


import java.util.function.DoubleSupplier;

import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmTeleop extends CommandBase {
  private final DoubleSupplier joystick; 
  private final Arm armSubsystem;
  /** Creates a new ArmTeleop. */
  public ArmTeleop(DoubleSupplier joystickSupplier, Arm armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    joystick = joystickSupplier;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }
  public void controllerMoveArm(DoubleSupplier rightJoystick) {
    /*
    // move the arm around based off the right joystick movement on the manipulator joystick
    //use the trapezoid thingy from robot code 2023
    //Math below | Summary: Take in controller Y axis as a double then calculate amount of volts needed to pass to the arm and when to stop based off of the controller movement. Done so by finding the constraints of the arm, translating the controller numbers, and finding how many volts and when to stop using feedvolts and PID
      double kgv = getKg();
      double armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(state.velocity, 0);
      double armPIDVolts = armPID.calculate(getArmPos(), state.position);
      if ((getArmPos() > ARM_UPPER_LIMIT_RAD && state.velocity > 0) ||
          (getArmPos() < ARM_LOWER_LIMIT_RAD && state.velocity < 0)) {
            forbFlag = true;  
          armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
          */

    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = joystick.getAsDouble();
    // use trapazoid math and controllerMoveArm method from arm subsytem to apply voltage to the motor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
