// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import java.util.function.DoubleSupplier;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopEffector extends Command {
  private final DoubleSupplier joystick;
  private final IntakeShooter intake;
  private final GenericHID manipulatorController;
  private final GenericHID driverController;
  private final Timer timer = new Timer();
  private boolean hasIntaked;

  /** Creates a new ArmTeleop. */
  public TeleopEffector(IntakeShooter effector, DoubleSupplier joystickSupplier, GenericHID manipulatorController, GenericHID driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake = effector);
    joystick = joystickSupplier;
    this.manipulatorController = manipulatorController;
    this.driverController = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    hasIntaked = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double joystickVal = joystick.getAsDouble();
    // if (joystickVal >= 0) {
    //   intake.setRPMIntake(MANUAL_RPM_MAX * joystick.getAsDouble());
    // } else if (joystickVal <= 0) {
    //   intake.setRPMIntake(MANUAL_RPM_MAX * joystick.getAsDouble());
    //   intake.setRPMOutake(MANUAL_RPM_MAX * joystick.getAsDouble());
    // }
    // manipulatorController.setRumble(RumbleType.kBothRumble, 0.5);
    
    if(intake.intakeDetectsNote() && intake.outtakeDetectsNote()){
      intake.setRPMOuttake(4000);    
      }
    else{
      intake.setRPMOuttake(0);    
    }  

    if (intake.intakeDetectsNote()) {
      // manipulatorController.setRumble(RumbleType.kBothRumble, 0.4);
      driverController.setRumble(RumbleType.kBothRumble, 0.4);
    } else if (intake.outtakeDetectsNote()) {
    manipulatorController.setRumble(RumbleType.kBothRumble, 0.4);
  } else {
    manipulatorController.setRumble(RumbleType.kBothRumble, 0);
    driverController.setRumble(RumbleType.kBothRumble, 0);
  }


    /*
     * intake.setRPMIntake(MANUAL_RPM_MAX * joystick.getAsDouble());
     * intake.setRPMOutake(MANUAL_RPM_MAX * joystick.getAsDouble());
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     // Reasoning: I WANNA TEST WHAT HAPPENS WHEN ANOTHER CMD
                                                                // IS USED OVER DEFAULT
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
