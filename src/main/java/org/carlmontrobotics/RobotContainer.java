// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

// java
import java.util.ArrayList;
import java.util.List;

// constants
import static org.carlmontrobotics.Constants.Armc.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import static org.carlmontrobotics.Constants.Effectorc.*;

// non static constants
import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.Constants.Armc;
import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Constants.Effectorc;
// robotcode2024 imports
import org.carlmontrobotics.commands.*;
import org.carlmontrobotics.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
//pathplanner
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

// wpilib geometry classes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

// controllers
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

// dashboards
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(Driver.port);
  public final GenericHID manipulatorController = new GenericHID(Manipulator.port);
  private final IntakeShooter intakeShooter = new IntakeShooter();

  // ignore warning, LED must be initialized
  private final Led led = new Led(intakeShooter);
  private final Arm arm = new Arm();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight(drivetrain);

  /* These are assumed to be equal to the AUTO ames in pathplanner */
    /* These must be equal to the pathPlanner path names from the GUI! */
  // Order matters - but the first one is index 1 on the physical selector - index 0 is reserved for null command.
  //the last auto is hard-coded to go straight. since we have __3__ Autos, port 4 is simple straight
  private List<Command> autoCommands = new ArrayList<Command>();
  private SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();

  private boolean hasSetupAutos = false;
  private final String[] autoNames = new String[] {
    /* These are assumed to be equal to the AUTO ames in pathplanner */
    "Left-Amp",
    "Center-Straight",
    "Middle-Ram",
    "Left-Straight",
    "Right-Straight",
    "Left-Safe"
  };
  DigitalInput[] autoSelectors = new DigitalInput[Math.min(autoNames.length, 10)];


  public RobotContainer() {
    {
      //safe auto setup... stuff in setupAutos() is not safe to run here - will break robot
      registerAutoCommands();
      SmartDashboard.putData(autoSelector);
      SmartDashboard.setPersistent("SendableChooser[0]");

      autoSelector.addOption("Nothing", 0);
      autoSelector.addOption("Raw Forward", 1);
      autoSelector.addOption("PP Simple Forward", 2);//index corresponds to index in autoCommands[]

      int i=3;
      for (String n:autoNames){
        autoSelector.addOption(n, i);
        i++;
      }

      ShuffleboardTab autoSelectorTab = Shuffleboard.getTab("Auto Chooser Tab");
      autoSelectorTab.add(autoSelector)
        .withSize(2, 1);
    }
      
    setDefaultCommands();
    setBindingsDriver();
    setBindingsManipulatorENDEFF();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(
        drivetrain,
        () -> ProcessedAxisValue(driverController, Axis.kLeftY),
        () -> ProcessedAxisValue(driverController, Axis.kLeftX),
        () -> ProcessedAxisValue(driverController, Axis.kRightX),
        () -> driverController.getRawButton(Driver.slowDriveButton)));
    // TODO: Are we going to use default command for intakeshooter?
    intakeShooter.setDefaultCommand(new TeleopEffector(
    intakeShooter,
    () -> ProcessedAxisValue(manipulatorController, Axis.kLeftY),
    manipulatorController, driverController
    ));

  }

  private void setBindingsDriver() {
    new JoystickButton(driverController, Driver.resetFieldOrientationButton)
        .onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
    new JoystickButton(driverController, 1).whileTrue(new AlignToApriltag(drivetrain)); // button A
    new JoystickButton(driverController, 2).whileTrue(new AlignToNote(drivetrain)); // button b?
    new JoystickButton(driverController, 3).whileTrue(new AutoMATICALLYGetNote(drivetrain, intakeShooter, limelight)); // button x?
    // new JoystickButton(driverController, OI.Driver.slowDriveButton).onTrue(new
    // ParallelCommandGroup(
    // new InstantCommand(()->drivetrain.setFieldOriented(false)),
    // new PrintCommand("Setting to ROBOT ORIENTED!!\nRO\nRO\nRO\n"))
    // ).onFalse(new ParallelCommandGroup(
    // new InstantCommand(()->drivetrain.setFieldOriented(true)),
    // new PrintCommand("Setting to FIELD FORI!!\nFO\nFO\nFO\n"))
    // );

    new JoystickButton(driverController, Driver.rotateFieldRelative0Deg)
        .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0), drivetrain));
    new JoystickButton(driverController, Driver.rotateFieldRelative90Deg)
        .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(270), drivetrain));
    new JoystickButton(driverController, Driver.rotateFieldRelative180Deg)
        .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180), drivetrain));
    new JoystickButton(driverController, Driver.rotateFieldRelative270Deg)
        .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
  }

  private void setBindingsManipulatorENDEFF() {
    /* /Eject also for AMP/ */
    new JoystickButton(manipulatorController, EJECT_BUTTON).onTrue(new Eject(intakeShooter));
    //new JoystickButton(manipulatorController, EJECT_BUTTON).onFalse(new InstantCommand());

    new JoystickButton(manipulatorController, AMP_BUTTON).onTrue(new PassToOutake(intakeShooter));
   // new JoystickButton(manipulatorController, AMP_BUTTON).onFalse(new InstantCommand());
    //new JoystickButton(manipulatorController, Button.kLeftBumper.value).onTrue(new OppositeEject(intakeShooter));
    axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
        .onTrue(
            new PassToOutake(intakeShooter));
    axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
        .onFalse(
            new InstantCommand(intakeShooter::stopOutake, intakeShooter));

    axisTrigger(manipulatorController, Manipulator.INTAKE_BUTTON)
        .onTrue(
            new Intake(intakeShooter));
    axisTrigger(manipulatorController, Manipulator.INTAKE_BUTTON)
        .onFalse(
            new InstantCommand(intakeShooter::stopIntake, intakeShooter));
    new JoystickButton(manipulatorController, Button.kY.value).onTrue(new MoveToPos(arm, AMP_ANGLE_RAD));
    new JoystickButton(manipulatorController, Button.kA.value).onTrue(new MoveToPos(arm, GROUND_INTAKE_POS));
    new JoystickButton(manipulatorController, Button.kB.value).onTrue(new ClimbArmSoftLimit(arm));
    new JoystickButton(manipulatorController, Button.kLeftStick.value).onTrue(new GETOUT(intakeShooter));
    new JoystickButton(manipulatorController, Button.kX.value).onTrue(new MoveToPos(arm, SPEAKER_ANGLE_RAD));
    // new JoystickButton(manipulatorController, Button.kB.value).onTrue(new
    // moveClimber(arm));
    // new JoystickButton(manipulatorController, Button.kX.value).onTrue(new
    // IntakeTesting(intakeShooter));
    // new WaitCommand(.5),
    // new MoveToPos(arm, GROUND_INTAKE_POS)
    // MoveToPos(arm, GROUND_INTAKE_POS));

    // new JoystickButton(manipulatorController, Button.kX.value).onTrue(new
    // MoveToPos(arm, Armc.UPPER_ANGLE_LIMIT_RAD));
    
    // NEW BINDINGS(easier for mani

    // Xbox left joy Y axis -> raw Intake/Outtake control
    // Xbox right joy Y axis -> raw Arm control
    // Xbox right trigger axis -> Intake pos + intake
    // Xbox left trigger axis -> amp pos , eject into amp
    // Xbox left bumper button -> CLOSE Speaker pos , Fire
    // Xbox right bumper button -> SAFE Speaker pos , Fire
    // Xbox X button -> goto Intake pos
    // Xbox Y button -> Eject rpm

    /* /Multi-commands/ */
    /*
     * axisTrigger(manipulatorController, OI.Manipulator.INTAKE_AX)
     * .onTrue(new SequentialCommandGroup(
     * new MoveToPos(arm, Armc.INTAKE_ANGLE_RAD),
     * new Intake(intakeShooter)
     * ));
     * /
     *//*
        * /
        * new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_CLOSE)//aka
        * podium
        * .onTrue(new SequentialCommandGroup(
        * new MoveToPos(arm, Armc.PODIUM_ANGLE_RAD),
        * new RampToRPM(intakeShooter, Effectorc.SUBWOOFER_RPM),
        * new PassToOutake(intakeShooter),
        * new WaitCommand(1),
        * new InstantCommand(intakeShooter::stopOutake)
        * ));
        * new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_SAFE)
        * .onTrue(new SequentialCommandGroup(
        * new MoveToPos(arm, Armc.SAFE_ZONE_ANGLE_RAD),
        * new RampToRPM(intakeShooter, Effectorc.SAFE_RPM),
        * new PassToOutake(intakeShooter),
        * new WaitCommand(1),
        * new InstantCommand(intakeShooter::stopOutake)
        * ));
        * axisTrigger(manipulatorController, OI.Manipulator.AMP_AX)//MELEE ATTACK
        * .onTrue(new SequentialCommandGroup(
        * //new MoveToPos(arm, Armc.AMP_ANGLE_RAD),
        * new Eject(intakeShooter)
        * ));
        * /
        *//*
           * /
           * new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_POS)
           * .onTrue(new MoveToPos(arm, Armc.INTAKE_ANGLE_RAD));
           * new JoystickButton(manipulatorController, OI.Manipulator.EJECT_RPM)
           * .onTrue(new Eject(intakeShooter));
           * new JoystickButton(manipulatorController, OI.Manipulator.RAISE_CLIMBER)
           * .onTrue(new MoveToPos(arm, Armc.CLIMBER_UP_ANGLE_RAD));
           * new JoystickButton(manipulatorController, OI.Manipulator.LOWER_CLIMBER)
           * .onTrue(new MoveToPos(arm, Armc.CLIMBER_DOWN_ANGLE_RAD));
           * // new JoystickButton(manipulatorController, Button.kLeftStick.value)
           * // .onTrue(new InstantCommand(() ->
           * {manipulatorController.setRumble(RumbleType.kBothRumble, 1);}));
           * 
           */
    // TODO: ask charles if passing in controller is okay
    // SmartDashboard.putData(new moveClimber(arm));
  }
  // private void setBindingsManipulatorARM() {
  // //NEW BINDINGS(easier for manipulator)
  // //Xbox left joy Y axis -> raw Intake/Outtake control
  // //Xbox right joy Y axis -> raw Arm control
  // //Xbox right trigger axis -> Intake pos + intake
  // //Xbox left trigger axis -> amp pos , eject into amp
  // //Xbox left bumper button -> CLOSE Speaker pos , Fire
  // //Xbox right bumper button -> SAFE Speaker pos , Fire
  // //Xbox X button -> goto Intake pos
  // //Xbox Y button -> Eject rpm

  // /*/Multi-commands/*/

  // axisTrigger(manipulatorController, OI.Manipulator.INTAKE_AX)
  // .onTrue(new SequentialCommandGroup(
  // new MoveToPos(arm, Armc.INTAKE_ANGLE_RAD),
  // new Intake(intakeShooter)
  // ));
  // /*//*/
  // new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_CLOSE)//aka
  // podium
  // .onTrue(new SequentialCommandGroup(
  // new MoveToPos(arm, Armc.PODIUM_ANGLE_RAD),
  // new RampToRPM(intakeShooter, Effectorc.SUBWOOFER_RPM),
  // new PassToOutake(intakeShooter),
  // new WaitCommand(1),
  // new InstantCommand(intakeShooter::stopOutake)
  // ));
  // new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_SAFE)
  // .onTrue(new SequentialCommandGroup(
  // new MoveToPos(arm, Armc.SAFE_ZONE_ANGLE_RAD),
  // new RampToRPM(intakeShooter, Effectorc.SAFE_RPM),
  // new PassToOutake(intakeShooter),
  // new WaitCommand(1),
  // new InstantCommand(intakeShooter::stopOutake)
  // ));
  // axisTrigger(manipulatorController, OI.Manipulator.AMP_AX)//MELEE ATTACK
  // .onTrue(new SequentialCommandGroup(
  // new MoveToPos(arm, Armc.AMP_ANGLE_RAD),
  // new Eject(intakeShooter)
  // ));
  // /*//*/
  // new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_POS)
  // .onTrue(new MoveToPos(arm, Armc.INTAKE_ANGLE_RAD));
  // new JoystickButton(manipulatorController, OI.Manipulator.EJECT_RPM)
  // .onTrue(new Eject(intakeShooter));
  // new JoystickButton(manipulatorController, OI.Manipulator.RAISE_CLIMBER)
  // .onTrue(new MoveToPos(arm, Armc.CLIMBER_UP_ANGLE_RAD));
  // new JoystickButton(manipulatorController, OI.Manipulator.LOWER_CLIMBER)
  // .onTrue(new MoveToPos(arm, Armc.CLIMBER_DOWN_ANGLE_RAD));
  // // new JoystickButton(manipulatorController, Button.kLeftStick.value)
  // // .onTrue(new InstantCommand(() ->
  // {manipulatorController.setRumble(RumbleType.kBothRumble, 1);}));

  // }
  

  /**
   * Flips an axis' Y coordinates upside down, but only if the select axis is a
   * joystick axis
   *
   * @param hid  The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double getStickValue(GenericHID hid, Axis axis) {
    return hid.getRawAxis(axis.value) * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
  }

  /**
   * Processes an input from the joystick into a value between -1 and 1,
   * sinusoidally instead of linearly
   *
   * @param value The value to be processed.
   * @return The processed value.
   */
  private double inputProcessing(double value) {
    double processedInput;
    // processedInput =
    // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
    processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2) * ((1 - Math.cos(value * Math.PI)) / 2),
        value);
    return processedInput;
  }

  /**
   * Combines both getStickValue and inputProcessing into a single function for
   * processing joystick outputs
   *
   * @param hid  The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double ProcessedAxisValue(GenericHID hid, Axis axis) {
    return DeadzonedAxis(inputProcessing(getStickValue(hid, axis)));
  }

  /**
   * Returns zero if a axis input is inside the deadzone
   *
   * @param hid  The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double DeadzonedAxis(double axOut) {
    return (Math.abs(axOut) <= OI.JOY_THRESH) ? 0.0 : axOut;
  }

  /**
   * Returns a new instance of Trigger based on the given Joystick and Axis
   * objects.
   * The Trigger is triggered when the absolute value of the stick value on the
   * specified axis
   * exceeds a minimum threshold value.
   *
   * @param stick The Joystick object to retrieve stick value from.
   * @param axis  The Axis object to retrieve value from the Joystick.
   * @return A new instance of Trigger based on the given Joystick and Axis
   *         objects.
*   * @throws NullPointerException if either stick or axis is null.
   */
  private Trigger axisTrigger(GenericHID controller, Axis axis) {
    return new Trigger(() -> Math.abs(getStickValue(controller, axis)) > OI.MIN_AXIS_TRIGGER_VALUE);
  }

  private void registerAutoCommands(){
    ////AUTO-USABLE COMMANDS
    NamedCommands.registerCommand("Intake", new Intake(intakeShooter));
    NamedCommands.registerCommand("Eject", new Eject(intakeShooter));

    NamedCommands.registerCommand("ArmToSpeakerSafe", new MoveToPos(arm, Armc.SAFE_ZONE_ANGLE_RAD));
    NamedCommands.registerCommand("ArmToSpeakerPodium", new MoveToPos(arm, Armc.PODIUM_ANGLE_RAD));
    NamedCommands.registerCommand("ArmToAmp", new MoveToPos(arm, Armc.AMP_ANGLE_RAD));

    // NamedCommands.registerCommand("RampRPMSpeakerSafe",
    //   new RampToRPM(intakeShooter, Effectorc.SAFE_RPM));
    // NamedCommands.registerCommand("RampRPMSpeakerSubwoofer",
    //   new RampToRPM(intakeShooter, Effectorc.SUBWOOFER_RPM));

    NamedCommands.registerCommand("PassToOutake", new PassToOutake(intakeShooter));
    NamedCommands.registerCommand("PassToIntake", new PassToIntake(intakeShooter));

    NamedCommands.registerCommand("StopIntake", new InstantCommand(intakeShooter::stopIntake));
    NamedCommands.registerCommand("StopOutake", new InstantCommand(intakeShooter::stopOutake));
    NamedCommands.registerCommand("StopBoth", new ParallelCommandGroup(
      new InstantCommand(intakeShooter::stopIntake),
      new InstantCommand(intakeShooter::stopOutake)
    ));
  }
  private void setupAutos() {
    ////CREATING PATHS from files
    {
      for (int i=0;i<autoNames.length;i++){
        String name = autoNames[i];

        autoCommands.add(new SequentialCommandGroup(new InstantCommand(() -> drivetrain.setPose(new Pose2d(1.92, 5.58, new Rotation2d(0.6)))), new PathPlannerAuto(name)));/*new SequentialCommandGroup(
          AutoBuilder.pathfindToPose(
            // PathPlannerAuto.getStaringPoseFromAutoFile(name),
            PathPlannerAuto.getPathGroupFromAutoFile(name).get(0).getPreviewStartingHolonomicPose(),
            Autoc.pathConstraints ),
          new PathPlannerAuto(name)
        ));*/
      }

      // ArrayList<PathPlannerPath> autoPaths = new ArrayList<PathPlannerPath>();
      // for (String name : autoNames) {
      //   autoPaths.add(PathPlannerPath.fromPathFile(name));
      // }


      // //AutoBuilder is setup in the drivetrain.

      // //note: is it .followPath or .buildAuto(name) + PathPlannerAuto​(autoName) ???
      // ////CREATE COMMANDS FROM PATHS
      // autoCommands = autoPaths.stream().map(
      //   (PathPlannerPath path) -> AutoBuilder.followPath(path)
      // ).collect(Collectors.toList());

    }


    //AUTOGENERATED AUTO FOR SLOT 2
    {
      Pose2d currPos = drivetrain.getPose();
      // Create a list of bezier points from poses. Each pose represents one waypoint.
      // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
              currPos,
              currPos.plus(new Transform2d(0,1,new Rotation2d(0)))
      );
      /**
       * PATHPLANNER SETTINGS
       * Robot Width (m): .91
       * Robot Length(m): .94
       * Max Module Spd (m/s): 4.30
       * Default Constraints
       * Max Vel: 1.54, Max Accel: 6.86
       * Max Angvel: 360, Max AngAccel: 360 (guesses!)
       */
      // Create the path using the bezier points created above
      PathPlannerPath path = new PathPlannerPath(
              bezierPoints,
              /*m/s, m/s^2, rad/s, rad/s^2 */
              Autoc.pathConstraints,
              new GoalEndState(0, currPos.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );
      // Prevent the path from being flipped if the coordinates are already correct
      path.preventFlipping = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
         path.flipPath();
      }

      //NOTHING
      autoCommands.add(0, new PrintCommand("Running NULL Auto!"));
      //RAW FORWARD command
      autoCommands.add(1, new LastResortAuto(drivetrain));
      //smart forward command
      autoCommands.add(2, new SequentialCommandGroup(
        new InstantCommand(() -> SmartDashboard.putNumber("starting x", path.getAllPathPoints().get(0).position.getX())),
        new InstantCommand(() -> SmartDashboard.putNumber("starting y", path.getAllPathPoints().get(0).position.getY())),
        new InstantCommand(() -> SmartDashboard.putNumber("wanted x", path.getAllPathPoints().get(path.getAllPathPoints().size() - 1).position.getX())),
        new InstantCommand(() -> SmartDashboard.putNumber("wanted y", path.getAllPathPoints().get(path.getAllPathPoints().size() - 1).position.getY())),
        AutoBuilder.followPath(path)
      ));//no events so just use path instead of auto

      // AutoBuilder.getAutoCommandFromJson((JSONObject) parser.parse(new FileReader("../deploy/pathplanner/autos/"+"Left-Straight"+".auto")));
    }
  }

  public Command getAutonomousCommand() {
    if (!hasSetupAutos){
      setupAutos();
      /*
      //get the funny ports on the robot
      for(int a = 0; a < autoSelectors.length; a++)
        autoSelectors[a] = new DigitalInput(a);//set up blank list
      */
      hasSetupAutos=true;
    }
    return autoCommands.get(autoSelector.getSelected()); //hard-coded PP straight auto
    // Integer autoIndex = autoSelector.getSelected();

    // if (autoIndex!=null && autoIndex!=0){
    //   new PrintCommand("Running selected auto: "+autoSelector.toString());
    //   return autoCommands.get(autoIndex.intValue());
    // }
    // new PrintCommand("No auto :(");
    // return null;

    /*

    //check which ones are short-circuiting
      for(int i = 2; i < autoSelectors.length; i++) { // skip index 0, reserved for null auto
        if(!autoSelectors[i].get()) {
          String name = autoNames[i-1];
          new PrintCommand("Using Path " + i + ": " + name);
          return new PathPlannerAuto(name);
        }
      }

    if (autoSelectors[1].get())//hard-coded straight auto at index 1
      return autoCommands.get(0);

    //return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoCommand;
    return new PrintCommand("No Auto selected | Auto selector broke :(");//nothing at index 0
    */        
	}

}