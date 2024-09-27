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
import static org.carlmontrobotics.Constants.Limelightc.*;

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
// pathplanner
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

// wpilib geometry classes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private static boolean babyMode = false;

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
    // Order matters - but the first one is index 1 on the physical selector - index
    // 0 is reserved for
    // null command.
    // the last auto is hard-coded to go straight. since we have __3__ Autos, port 4
    // is simple
    // straight
    private List<Command> autoCommands;
    private SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();

    private boolean hasSetupAutos = false;
    private final String[] autoNames = new String[] {
            /* These are assumed to be equal to the AUTO ames in pathplanner */
            "Left-Auto Ruiner", "Center-Auto Ruiner", "Right-Auto Ruiner",
            "Right Limelight 4 Piece", "Left Limelight 4 Piece",
            "Center Limelight 4 Piece",

            "Left-Amp",

            "Center Limelight 1 Piece", "Left Limelight 1 Piece",
            "Right Limelight 1 Piece", "Center Field Limelight",
            "Center Field Limelight(No Preload)", "Center Forward",
            "Right Forward", "Left Forward",
            "Backup-Center", "Backup-Right", "Backup-Left",
                    "Preload1Center", "Preload1Right", "Preload1Left",

    };
    DigitalInput[] autoSelectors = new DigitalInput[Math.min(autoNames.length, 10)];

    public RobotContainer() {
        {
            // Put any configuration overrides to the dashboard and the terminal
            SmartDashboard.putData("CONFIG overrides", Config.CONFIG);
            System.out.println(Config.CONFIG);

            SmartDashboard.putData("BuildConstants", BuildInfo.getInstance());

            SmartDashboard.setDefaultBoolean("babymode", babyMode);
            SmartDashboard.setPersistent("babymode");
            // safe auto setup... stuff in setupAutos() is not safe to run here - will break
            // robot
            registerAutoCommands();
            SmartDashboard.putData(autoSelector);
            SmartDashboard.setPersistent("SendableChooser[0]");

            autoSelector.addOption("Nothing", 0);
            autoSelector.addOption("Raw Forward", 1);
            autoSelector.addOption("PP Simple Forward", 2);// index corresponds to index in autoCommands[]

            int i = 3;
            for (String n : autoNames) {
                autoSelector.addOption(n, i);
                i++;
            }

            ShuffleboardTab autoSelectorTab = Shuffleboard.getTab("Auto Chooser Tab");
            autoSelectorTab.add(autoSelector).withSize(2, 1);
        }

        setDefaultCommands();
        setBindingsDriver();
        // setBindingsManipulatorENDEFF();
        setBindingsManipulator();
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain,
                () -> ProcessedAxisValue(driverController, Axis.kLeftY),
                () -> ProcessedAxisValue(driverController, Axis.kLeftX),
                () -> ProcessedAxisValue(driverController, Axis.kRightX),
                () -> driverController.getRawButton(Driver.slowDriveButton)));
        // TODO: Are we going to use default command for intakeshooter?
        intakeShooter.setDefaultCommand(new TeleopEffector(intakeShooter,
                () -> ProcessedAxisValue(manipulatorController, Axis.kLeftY),
                manipulatorController, driverController));
        // TODO
        // intakeShooter.setDefaultCommand(new RampMaxRPMDriving(intakeShooter));

        arm.setDefaultCommand(
                Config.CONFIG.useSmartDashboardControl ? new TestArmToPos(arm)
                        : new TeleopArm(arm,
                                () -> ProcessedAxisValue(manipulatorController,
                                        Axis.kLeftY)));

    }

    private void setBindingsDriver() {
        new JoystickButton(driverController, Driver.resetFieldOrientationButton)
                .onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
        // axisTrigger(driverController, Axis.kRightTrigger)
        // .whileTrue(new SequentialCommandGroup(new PrintCommand("Running Intake"),
        // new AutoMATICALLYGetNote(drivetrain, intakeShooter, limelight)));

        new POVButton(driverController, 0)
                .whileTrue(new ParallelCommandGroup(new Intake(intakeShooter),
                        new AutoMATICALLYGetNote(drivetrain, limelight,
                                intakeShooter, 1)));

        axisTrigger(driverController, Axis.kLeftTrigger)
                // .onTrue(new AlignToApriltag(drivetrain, limelight));
                .onTrue(new InstantCommand(() -> drivetrain.setFieldOriented(false)))
                .onFalse(new InstantCommand(() -> drivetrain.setFieldOriented(true)));

        axisTrigger(driverController, Manipulator.SHOOTER_BUTTON)
                .whileTrue(new AlignToApriltag(drivetrain, limelight, 2.0));
        new JoystickButton(driverController, Driver.rotateFieldRelative0Deg).onTrue(
                new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0), drivetrain));
        new JoystickButton(driverController, Driver.rotateFieldRelative90Deg)
                .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(270),
                        drivetrain));
        new JoystickButton(driverController, Driver.rotateFieldRelative180Deg)
                .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180),
                        drivetrain));
        new JoystickButton(driverController, Driver.rotateFieldRelative270Deg)
                .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90),
                        drivetrain));
    }

    private void setBindingsManipulator() {
        new JoystickButton(manipulatorController, EJECT_BUTTON)
                .onTrue(new Eject(intakeShooter));

        // new JoystickButton(manipulatorController, A_BUTTON)
        // .onTrue(new RampMaxRPMDriving(intakeShooter));
        // axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON).whileTrue(
        // new SequentialCommandGroup(new AimArmSpeaker(arm, limelight),
        // new PassToOuttake(intakeShooter)));

        axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON).whileTrue(
                new ConditionalCommand(new SequentialCommandGroup(new AimArmSpeaker(arm, limelight),
                        new PassToOuttake(intakeShooter)), new InstantCommand(() -> {
                        }), () -> LimelightHelpers.getTV(SHOOTER_LL_NAME)));

        // axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
        // .whileTrue(new PassToOuttake(intakeShooter));

        // axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
        // .whileTrue(new AimArmSpeaker(arm, limelight));

        new JoystickButton(manipulatorController, RAMP_OUTTAKE)
                .whileTrue(new RampMaxRPM(intakeShooter));
        new JoystickButton(manipulatorController, OPPOSITE_EJECT)
                .whileTrue(new EjectOuttakeSide(intakeShooter));

        /*
         * axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
         * .onTrue(Config.CONFIG.useSmartDashboardControl
         * ? new TestRPM(intakeShooter)
         * : new Outtake(intakeShooter, arm)
         */

        // axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
        // .onTrue(new PassToOuttake(intakeShooter));

        axisTrigger(manipulatorController, Manipulator.INTAKE_BUTTON)
                .whileTrue(new Intake(intakeShooter));
        new JoystickButton(manipulatorController, ARM_TO_AMP_BUTTON)
                .onTrue(new ArmToPos(arm, AMP_ANGLE_RAD_NEW_MOTOR));
        new JoystickButton(manipulatorController, A_BUTTON)
                .onTrue(new ArmToPos(arm, GROUND_INTAKE_POS));
        new JoystickButton(manipulatorController, PASS_TO_OUTTAKE_STICK)
                .onTrue(new PassToOuttake(intakeShooter));
        new JoystickButton(manipulatorController, PASS_TO_INTAKE_STICK)
                .onTrue(new PassToIntake(intakeShooter));
        new JoystickButton(manipulatorController, SPEAKER_POS)
                .onTrue(new ArmToPos(arm, SPEAKER_ANGLE_RAD));
        new POVButton(manipulatorController, UP_D_PAD)
                .onTrue(new ArmToPos(arm, CLIMB_POS));
        new POVButton(manipulatorController, DOWN_D_PAD).onTrue(new Climb(arm));
        new POVButton(manipulatorController, LEFT_D_PAD)
                .onTrue(new ArmToPos(arm, PODIUM_ANGLE_RAD));

    }

    /**
     * Flips an axis' Y coordinates upside down, but only if the select axis is a
     * joystick axis
     *
     * @param hid  The controller/plane joystick the axis is on
     * @param axis The processed axis
     * @return The processed value.
     */
    private double getStickValue(GenericHID hid, Axis axis) {
        return hid.getRawAxis(axis.value)
                * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
    }

    /**
     * Processes an input from the joystick into a value between -1 and 1,
     * sinusoidally instead of
     * linearly
     *
     * @param value The value to be processed.
     * @return The processed value.
     */
    private double inputProcessing(double value) {
        double processedInput;
        // processedInput =
        // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
        processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2)
                * ((1 - Math.cos(value * Math.PI)) / 2), value);
        return processedInput;
    }

    /**
     * Combines both getStickValue and inputProcessing into a single function for
     * processing joystick
     * outputs
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
     * objects. The Trigger is
     * triggered when the absolute value of the stick value on the specified axis
     * exceeds a minimum
     * threshold value.
     *
     * @param stick The Joystick object to retrieve stick value from.
     * @param axis  The Axis object to retrieve value from the Joystick.
     * @return A new instance of Trigger based on the given Joystick and Axis
     *         objects. * @throws
     *         NullPointerException if either stick or axis is null.
     */
    private Trigger axisTrigger(GenericHID controller, Axis axis) {
        return new Trigger(() -> Math
                .abs(getStickValue(controller, axis)) > OI.MIN_AXIS_TRIGGER_VALUE);
    }

    private void registerAutoCommands() {
        //// AUTO-USABLE COMMANDS
        NamedCommands.registerCommand("Intake", new Intake(intakeShooter));
        NamedCommands.registerCommand("Eject", new Eject(intakeShooter));

        // NamedCommands.registerCommand("ArmToSpeaker", new MoveToPos(arm,
        // Armc.SPEAKER_ANGLE_RAD, 0));
        NamedCommands.registerCommand("ArmToAmp",
                new ArmToPos(arm, Armc.AMP_ANGLE_RAD));
        NamedCommands.registerCommand("ArmToSubwoofer",
                new ArmToPos(arm, Armc.SUBWOOFER_ANGLE_RAD));
        NamedCommands.registerCommand("ArmToPodium",
                new ArmToPos(arm, Armc.PODIUM_ANGLE_RAD));
        NamedCommands.registerCommand("ArmToGround",
                new ArmToPos(arm, GROUND_INTAKE_POS));

        NamedCommands.registerCommand("RampRPMAuton",
                new RampRPMAuton(intakeShooter));

        NamedCommands.registerCommand("SwitchRPMShoot",
                new Outtake(intakeShooter, arm));

        // NamedCommands.registerCommand("AutonRuinerShoot", new
        // AutonRuinerShoot(intakeShooter));
        // NamedCommands.registerCommand("IntakeAutonRuiner", new
        // IntakeAutonRuiner(intakeShooter));

        NamedCommands.registerCommand("AutonRuinerShootAndIntake",
                new AutonRuinerShootAndIntake(intakeShooter, arm));

        NamedCommands.registerCommand("PassToOuttake",
                new PassToOuttake(intakeShooter));
        NamedCommands.registerCommand("AimArmSpeakerMT2",
                new AimArmSpeaker(arm, limelight));
        NamedCommands.registerCommand("AlignToAprilTagMegaTag2",
                new AlignToApriltag(drivetrain, limelight, 0.0));
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(3.0),
                        new SequentialCommandGroup(
                                // TODO: Use Align To Drivetrain
                                // new AlignDrivetrain(drivetrain),
                                new ParallelCommandGroup(
                                        new AlignToApriltag(drivetrain, limelight, 0.0),
                                        new AimArmSpeaker(arm, limelight),
                                        new RampRPMAuton(intakeShooter)),
                                new PassToOuttake(intakeShooter),
                                new ArmToPos(arm, GROUND_INTAKE_POS)))));
        NamedCommands.registerCommand("ShootSubwoofer",
                        new SequentialCommandGroup(new ParallelCommandGroup(
                                        new ArmToPos(arm,
                                                        Armc.SUBWOOFER_ANGLE_RAD),
                                        new RampRPMAuton(intakeShooter)),
                        new PassToOuttake(intakeShooter),
                        new ArmToPos(arm, GROUND_INTAKE_POS)));
        NamedCommands.registerCommand("Limelight Intake CCW",
                new ParallelCommandGroup(new Intake(intakeShooter),
                        new AutoMATICALLYGetNote(drivetrain, limelight,
                                intakeShooter, 1)));
        NamedCommands.registerCommand("Limelight Intake CW",
                new ParallelCommandGroup(new Intake(intakeShooter),
                        new AutoMATICALLYGetNote(drivetrain, limelight,
                                intakeShooter, -1)));

        NamedCommands.registerCommand("Limelight Intake Straight",
                new ParallelCommandGroup(new Intake(intakeShooter),
                        new AutoMATICALLYGetNote(drivetrain, limelight,
                                intakeShooter, 0)));

        NamedCommands.registerCommand("StopIntake",
                new InstantCommand(intakeShooter::stopIntake));
        NamedCommands.registerCommand("StopOutake",
                new InstantCommand(intakeShooter::stopOuttake));
        NamedCommands.registerCommand("StopBoth",
                new ParallelCommandGroup(new InstantCommand(intakeShooter::stopIntake),
                        new InstantCommand(intakeShooter::stopOuttake)));
    }

    private void setupAutos() {
        //// CREATING PATHS from files
        if (!hasSetupAutos) {
            autoCommands=new ArrayList<Command>();//clear old/nonexistent autos

            for (int i = 0; i < autoNames.length; i++) {
                String name = autoNames[i];

                autoCommands.add(new PathPlannerAuto(name));

                /*
                 * // Charles' opinion: we shouldn't have it path find to the starting pose at the start of match
                 * new SequentialCommandGroup( 
                 *      AutoBuilder.pathfindToPose(
                 *          PathPlannerAuto.getStaringPoseFromAutoFile(name),
                 *          PathPlannerAuto.getPathGroupFromAutoFile(name).get(0).
                 *          getPreviewStartingHolonomicPose(),
                 *          Autoc.pathConstraints), 
                 *      new PathPlannerAuto(name));
                 */
            }
            hasSetupAutos = true;

            // NOTHING
            autoCommands.add(0, new PrintCommand("Running NULL Auto!"));
            // RAW FORWARD command
            autoCommands.add(1, new SequentialCommandGroup(
                            new InstantCommand(() -> drivetrain.drive(-.0001, 0, 0)), new WaitCommand(0.5),
                            new LastResortAuto(drivetrain)));
            // dumb PP forward command
            autoCommands.add(2, new PrintCommand("PPSimpleAuto not Configured!"));
        }
        // force regeneration each auto call
        autoCommands.set(2, constructPPSimpleAuto());//overwrite this slot each time auto runs
    }

    public Command constructPPSimpleAuto() {
        /**
         * PATHPLANNER SETTINGS Robot Width (m): .91 Robot Length(m): .94 Max Module Spd
         * (m/s): 4.30
         * Default Constraints Max Vel: 1.54, Max Accel: 6.86 Max Angvel: 360, Max
         * AngAccel: 360
         * (guesses!)
         */
        // default origin is on BLUE ALIANCE DRIVER RIGHT CORNER
        Pose2d currPos = drivetrain.getPose(); 

        //FIXME running red PP file autos seems to break something, so the robot drivetrain drives in the wrong direction.
            //running blue PP autos is fine though
        //Note: alliance detection and path generation work correctly!
        //Solution: Redeploy after auto.
        Pose2d endPos = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                        ? currPos.transformBy(new Transform2d(1, 0, new Rotation2d(0)))
                        : currPos.transformBy(new Transform2d(-1, 0, new Rotation2d(0)));

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(currPos, endPos);

        // Create the path using the bezier points created above, /* m/s, m/s^2, rad/s, rad/s^2 */
        PathPlannerPath path = new PathPlannerPath(bezierPoints,
                Autoc.pathConstraints, new GoalEndState(0, currPos.getRotation()));
        
        path.preventFlipping = false;// don't flip, we do that manually already.

        return new SequentialCommandGroup(
            new InstantCommand(()->drivetrain.drive(-.0001, 0, 0)),//align drivetrain wheels.
            AutoBuilder.followPath(path).beforeStarting(new WaitCommand(1)));
    }

    public Command getAutonomousCommand() {
        setupAutos();

        Integer autoIndex = autoSelector.getSelected();

        if (autoIndex != null && autoIndex != 0) {
            new PrintCommand("Running selected auto: " + autoSelector.toString());
            return autoCommands.get(autoIndex.intValue());
        }
        return new PrintCommand("No auto :(");
    }

}
