// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//import org.carlmontrobotics.subsystems.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);
  //Intake intake =  new Intake();
  public RobotContainer() {
    /*new JoystickButton(controller, Button.kX.value).whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(controller, Button.kA.value).whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(controller, Button.kB.value).whileTrue(intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    new JoystickButton(controller, Button.kY.value).whileTrue(intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
*/
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
