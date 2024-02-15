package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Drivetrain.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SensorFactory;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.swerve.SwerveModule;
import org.carlmontrobotics.commands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.commands.TeleopDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

//fuckit
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.examples.rapidreactcommandbot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class Drivetrain extends SubsystemBase {
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2

    private SwerveDriveKinematics kinematics = null;
    private SwerveDriveOdometry odometry = null;
    private SwerveModule modules[];
    private boolean fieldOriented = true;
    private double fieldOffset = 0;
    private CANSparkMax[] driveMotors;
    // gyro
    public final float initPitch;
    public final float initRoll;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage>[] m_appliedVoltage = new MutableMeasure[4];
    
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance>[] m_distance = new MutableMeasure[4];
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>>[] m_velocity = new MutableMeasure[4];

    private enum SysIdTest {
        FRONT_DRIVE,
        BACK_DRIVE,
        ALL_DRIVE
    }

    private SendableChooser<SysIdTest> sysIdChooser = new SendableChooser<>();

    // Create a new SysId routine for characterizing the drive.
    private SysIdRoutine frontOnlyRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to give the driving voltage to the motors.
            (Measure<Voltage> volts) -> {
                driveMotors[0].setVoltage(volts.in(Volts));
                driveMotors[1].setVoltage(volts.in(Volts));
                modules[2].coast();
                modules[3].coast();                                                     
            },
            log -> {// FRONT
                log.motor("fl")
                    .voltage(m_appliedVoltage[0].mut_replace(
                            driveMotors[0].getBusVoltage() * driveMotors[0].getAppliedOutput(), Volts))
                    .linearPosition(
                            m_distance[0].mut_replace(driveMotors[0].getEncoder().getPosition(), Meters))
                    .linearVelocity(m_velocity[0].mut_replace(driveMotors[0].getEncoder().getVelocity(),
                            MetersPerSecond));
                log.motor("fr")
                    .voltage(m_appliedVoltage[1].mut_replace(
                            driveMotors[1].getBusVoltage() * driveMotors[1].getAppliedOutput(), Volts))
                    .linearPosition(
                            m_distance[1].mut_replace(driveMotors[1].getEncoder().getPosition(), Meters))
                    .linearVelocity(m_velocity[1].mut_replace(driveMotors[1].getEncoder().getVelocity(),
                            MetersPerSecond));
            },
            this
        )
    );

    private SysIdRoutine backOnlyRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                modules[0].coast();
                modules[1].coast();
                driveMotors[2].setVoltage(volts.in(Volts));
                driveMotors[3].setVoltage(volts.in(Volts));
            },
            log -> {// BACK
                log.motor("bl")
                    .voltage(m_appliedVoltage[2].mut_replace(
                            driveMotors[2].getBusVoltage() * driveMotors[2].getAppliedOutput(), Volts))
                    .linearPosition(
                            m_distance[2].mut_replace(driveMotors[2].getEncoder().getPosition(), Meters))
                    .linearVelocity(m_velocity[2].mut_replace(driveMotors[2].getEncoder().getVelocity(),
                            MetersPerSecond));
                log.motor("br")
                    .voltage(m_appliedVoltage[3].mut_replace(
                            driveMotors[3].getBusVoltage() * driveMotors[3].getAppliedOutput(), Volts))
                    .linearPosition(
                            m_distance[3].mut_replace(driveMotors[3].getEncoder().getPosition(), Meters))
                    .linearVelocity(m_velocity[3].mut_replace(driveMotors[3].getEncoder().getVelocity(),
                            MetersPerSecond));
            },
            this
        )
    );

    private SysIdRoutine allWheelsRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                for (CANSparkMax dm : driveMotors) {
                    dm.setVoltage(volts.in(Volts));
                }
            }, 
            log -> {
                log.motor("fl")
                        .voltage(m_appliedVoltage[0].mut_replace(
                                driveMotors[0].getBusVoltage() * driveMotors[0].getAppliedOutput(), Volts))
                        .linearPosition(
                                m_distance[0].mut_replace(driveMotors[0].getEncoder().getPosition(), Meters))
                        .linearVelocity(m_velocity[0].mut_replace(driveMotors[0].getEncoder().getVelocity(),
                                MetersPerSecond));
                log.motor("fr")
                        .voltage(m_appliedVoltage[1].mut_replace(
                                driveMotors[1].getBusVoltage() * driveMotors[1].getAppliedOutput(), Volts))
                        .linearPosition(
                                m_distance[1].mut_replace(driveMotors[1].getEncoder().getPosition(), Meters))
                        .linearVelocity(m_velocity[1].mut_replace(driveMotors[1].getEncoder().getVelocity(),
                                MetersPerSecond));
                log.motor("bl")
                        .voltage(m_appliedVoltage[2].mut_replace(
                                driveMotors[2].getBusVoltage() * driveMotors[2].getAppliedOutput(), Volts))
                        .linearPosition(
                                m_distance[2].mut_replace(driveMotors[2].getEncoder().getPosition(), Meters))
                        .linearVelocity(m_velocity[2].mut_replace(driveMotors[2].getEncoder().getVelocity(),
                                MetersPerSecond));
                log.motor("br")
                        .voltage(m_appliedVoltage[3].mut_replace(
                                driveMotors[3].getBusVoltage() * driveMotors[3].getAppliedOutput(), Volts))
                        .linearPosition(
                                m_distance[3].mut_replace(driveMotors[3].getEncoder().getPosition(), Meters))
                        .linearVelocity(m_velocity[3].mut_replace(driveMotors[3].getEncoder().getVelocity(),
                                MetersPerSecond));
            }, 
            this
        )
    );

    private ShuffleboardTab sysIdTab = Shuffleboard.getTab("Drivetrain SysID");

    // debug purposes
    private SwerveModule moduleFL;
    private SwerveModule moduleFR;
    private SwerveModule moduleBL;
    private SwerveModule moduleBR;

    public Drivetrain() {
        // Calibrate Gyro
        {
            double initTimestamp = Timer.getFPGATimestamp();
            double currentTimestamp = initTimestamp;
            while (gyro.isCalibrating() && currentTimestamp - initTimestamp < 10) {
                currentTimestamp = Timer.getFPGATimestamp();
                try {
                    Thread.sleep(1000);// 1 second
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    break;
                }
                System.out.println("Calibrating the gyro...");
            }
            gyro.reset();
            System.out.println("NavX-MXP firmware version: " + gyro.getFirmwareVersion());
            System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());
        }

        // Setup Kinematics
        {
            // Define the corners of the robot relative to the center of the robot using
            // Translation2d objects.
            // Positive x-values represent moving toward the front of the robot whereas
            // positive y-values represent moving toward the left of the robot.
            Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
            Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
            Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
            Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

            kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        }

        // Initialize modules
        {
            // initPitch = 0;
            // initRoll = 0;
            Supplier<Float> pitchSupplier = () -> 0F;
            Supplier<Float> rollSupplier = () -> 0F;
            initPitch = gyro.getPitch();
            initRoll = gyro.getRoll();
            // Supplier<Float> pitchSupplier = () -> gyro.getPitch();
            // Supplier<Float> rollSupplier = () -> gyro.getRoll();

            driveMotors = new CANSparkMax[4];

            moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL,
                    driveMotors[0] = MotorControllerFactory.createSparkMax(driveFrontLeftPort, MotorConfig.NEO),
                    MotorControllerFactory.createSparkMax(turnFrontLeftPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortFL), 0,
                    pitchSupplier, rollSupplier);
            // Forward-Right
            moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR,
                    driveMotors[1] = MotorControllerFactory.createSparkMax(driveFrontRightPort, MotorConfig.NEO),
                    MotorControllerFactory.createSparkMax(turnFrontRightPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortFR), 1,
                    pitchSupplier, rollSupplier);

            // Backward-Left
            moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL,
                    driveMotors[2] = MotorControllerFactory.createSparkMax(driveBackLeftPort, MotorConfig.NEO),
                    MotorControllerFactory.createSparkMax(turnBackLeftPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortBL), 2,
                    pitchSupplier, rollSupplier);
            // Backward-Right
            moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR,
                    driveMotors[3] = MotorControllerFactory.createSparkMax(driveBackRightPort, MotorConfig.NEO),
                    MotorControllerFactory.createSparkMax(turnBackRightPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortBR), 3,
                    pitchSupplier, rollSupplier);
            modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
            for (CANSparkMax driveMotor : driveMotors) {
                driveMotor.setOpenLoopRampRate(secsPer12Volts);

                driveMotor.getEncoder().setPositionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
                driveMotor.getEncoder().setVelocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
            }
            // for(CANSparkMax driveMotor : driveMotors)
            // driveMotor.setSmartCurrentLimit(80);
        }

        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(),
                new Pose2d());


        // SysId Setup
        {
            Supplier<SequentialCommandGroup> stopNwait = ()->new SequentialCommandGroup(new InstantCommand(this::stop), new WaitCommand(2));

            /* Alex's old sysId tests 
            sysIdTab.add("All sysid tests", new SequentialCommandGroup(
                new SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kForward,2), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kReverse,2), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kForward,2), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kReverse,2), (Command)stopNwait.get())
            ));
            sysIdTab.add("All sysid tests - FRONT wheels", new SequentialCommandGroup(
                new SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kForward,0), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kReverse,0), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kForward,0), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kReverse,0), (Command)stopNwait.get())
            ));
            sysIdTab.add("All sysid tests - BACK wheels", new SequentialCommandGroup(
                new SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kForward,1), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kReverse,1), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kForward,1), (Command)stopNwait.get()),
                new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kReverse,1), (Command)stopNwait.get())
            ));
            */
            sysIdTab
                .add("Run Quasistatic Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward))
                .withSize(2, 1);
            sysIdTab
                .add("Run Quasistatic Backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                .withSize(2, 1);
            sysIdTab
                .add("Run Dynamic Forward", sysIdDynamic(SysIdRoutine.Direction.kForward))
                .withSize(2, 1);
            sysIdTab
                .add("Run Dynamic Backward", sysIdDynamic(SysIdRoutine.Direction.kReverse))
                .withSize(2, 1);

            sysIdChooser.addOption("Front Only Drive", SysIdTest.FRONT_DRIVE);
            sysIdChooser.addOption("Back Only Drive", SysIdTest.BACK_DRIVE);
            sysIdChooser.addOption("All Wheels Drive", SysIdTest.ALL_DRIVE);
            sysIdTab
                .add(sysIdChooser)
                .withSize(2, 1);

            sysIdTab.add(this);

            for (int i = 0; i < 4; i++) {
                m_appliedVoltage[i] = mutable(Volts.of(0));
                m_distance[i] = mutable(Meters.of(0));
                m_velocity[i] = mutable(MetersPerSecond.of(0));
            }
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, int frontorback) {
        switch(frontorback) {
            case 0:
                return frontOnlyRoutine.quasistatic(direction);
            case 1:
                return backOnlyRoutine.quasistatic(direction);
            case 2:
                return allWheelsRoutine.quasistatic(direction);
        }
        return new PrintCommand("Invalid Command");
    }

    private SysIdTest selector() {
        SysIdTest test = sysIdChooser.getSelected();
        System.out.println("Test Selected: " + test);
        return test;
    }
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running front only quasistatic forward") :
                        new PrintCommand("Running front only quasistatic backward"), 
                    frontOnlyRoutine.quasistatic(direction)
                )),
                Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running back only quasistatic forward") :
                        new PrintCommand("Running back only quasistatic backward"), 
                    backOnlyRoutine.quasistatic(direction)
                )),
                Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running all wheels quasistatic forward") :
                        new PrintCommand("Running all wheels quasistatic backward"), 
                    allWheelsRoutine.quasistatic(direction)
                ))
            ),
            this::selector
        );
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, int frontorback) {
        switch(frontorback) {
            case 0:
                return frontOnlyRoutine.dynamic(direction);
            case 1:
                return backOnlyRoutine.dynamic(direction);
            case 2:
                return allWheelsRoutine.dynamic(direction);
        }
        return new PrintCommand("Invalid Command");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running front only dynamic forward") :
                        new PrintCommand("Running front only dynamic backward"), 
                    frontOnlyRoutine.dynamic(direction)
                )),
                Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running back only dynamic forward") :
                        new PrintCommand("Running back only dynamic backward"),
                    backOnlyRoutine.dynamic(direction)
                )),
                Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running all wheels dynamic forward") :
                        new PrintCommand("Running all wheels dynamic backward"),
                    allWheelsRoutine.dynamic(direction)
                ))
            ),
            this::selector
        );
    }

    @Override
    public void periodic() {
        // lobotomized to prevent ucontrollabe swerve behavior
        // FIXME: unlobotomize lib199
        for (SwerveModule module : modules)
            module.periodic();

        // Update the odometry with current heading and encoder position
        odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        autoCancelDtCommand();

        // SmartDashboard.putNumber("Odometry X", getPose().getTranslation().getX());
        // SmartDashboard.putNumber("Odometry Y", getPose().getTranslation().getY());
        // // SmartDashboard.putNumber("Pitch", gyro.getPitch());
        // // SmartDashboard.putNumber("Roll", gyro.getRoll());
        // SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // // SmartDashboard.putNumber("AdjRoll", gyro.getPitch() - initPitch);
        // // SmartDashboard.putNumber("AdjPitch", gyro.getRoll() - initRoll);
        // SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        // SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        // SmartDashboard.putNumber("Compass Offset", compassOffset);
        // SmartDashboard.putBoolean("Current Magnetic Field Disturbance",
        // gyro.isMagneticDisturbance());
        // SmartDashboard.putNumber("front left encoder", moduleFL.getModuleAngle());
        // SmartDashboard.putNumber("front right encoder", moduleFR.getModuleAngle());
        // SmartDashboard.putNumber("back left encoder", moduleBL.getModuleAngle());
        // SmartDashboard.putNumber("back right encoder", moduleBR.getModuleAngle());

    }

    public void autoCancelDtCommand() {
        if (!(getDefaultCommand() instanceof TeleopDrive) || DriverStation.isAutonomous())
            return;

        // Use hasDriverInput to get around acceleration limiting on slowdown
        if (((TeleopDrive) getDefaultCommand()).hasDriverInput()) {
            Command currentDtCommand = getCurrentCommand();
            if (currentDtCommand != getDefaultCommand() && !(currentDtCommand instanceof RotateToFieldRelativeAngle)
                    && currentDtCommand != null) {
                currentDtCommand.cancel();
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // for (SwerveModule module : modules)
        //     SendableRegistry.addChild(this, module);

        // builder.addBooleanProperty("Magnetic Field Disturbance", gyro::isMagneticDisturbance, null);
        // builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
        // builder.addBooleanProperty("Field Oriented", () -> fieldOriented,
        //         fieldOriented -> this.fieldOriented = fieldOriented);
        // builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
        // builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
        // builder.addDoubleProperty("Odometry Heading", () -> getPose().getRotation().getDegrees(), null);
        // builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
        // builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
        // builder.addDoubleProperty("Pitch", gyro::getPitch, null);
        // builder.addDoubleProperty("Roll", gyro::getRoll, null);
        // builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset -> this.fieldOffset = fieldOffset);
    }

    // #region Drive Methods

    /**
     * Drives the robot using the given x, y, and rotation speed
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive
     */
    public void drive(double forward, double strafe, double rotation) {
        drive(getSwerveStates(forward, strafe, rotation));
    }

    public void drive(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

        // Move the modules based on desired (normalized) speed, desired angle, max
        // speed, drive modifier, and whether or not to reverse turning.
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("moduleIn" + Integer.toString(i), moduleStates[i].angle.getDegrees());
            // moduleStates[i] = SwerveModuleState.optimize(moduleStates[i],
            // Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            SmartDashboard.putNumber("moduleOT" + Integer.toString(i), moduleStates[i].angle.getDegrees());

            modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }

    public void stop() {
        for (SwerveModule module : modules)
            module.move(0, 0);
    }

    public boolean isStopped() {
        return Math.abs(getSpeeds().vxMetersPerSecond) < 0.1 &&
                Math.abs(getSpeeds().vyMetersPerSecond) < 0.1 &&
                Math.abs(getSpeeds().omegaRadiansPerSecond) < 0.1;
    }

    /**
     * Constructs and returns a ChassisSpeeds objects using forward, strafe, and
     * rotation values.
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive.
     * @return A ChassisSpeeds object.
     */
    private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation,
                    Rotation2d.fromDegrees(getHeading()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, rotation);
        }
        return speeds;
    }

    /**
     * Constructs and returns four SwerveModuleState objects, one for each side,
     * using forward, strafe, and rotation values.
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive.
     * @return A SwerveModuleState array, one for each side of the drivetrain (FL,
     *         FR, etc.).
     */
    private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
        return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, rotation));
    }

    // #endregion

    // #region Getters and Setters

    // returns a value from -180 to 180
    public double getHeading() {
        double x = gyro.getAngle();
        if (fieldOriented)
            x -= fieldOffset;
        return Math.IEEEremainder(x * (isGyroReversed ? -1.0 : 1.0), 360);
    }

    public double getHeadingDeg() {
        return getHeading();
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getCurrentPosition).toArray(SwerveModulePosition[]::new);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d initialPose) {
        odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), initialPose);
    }

    // Resets the gyro, so that the direction the robotic currently faces is
    // considered "forward"
    public void resetHeading() {
        gyro.reset();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public boolean getFieldOriented() {
        return fieldOriented;
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    public void resetFieldOrientation() {
        fieldOffset = gyro.getAngle();
    }

    public void resetOdometry() {
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
        gyro.reset();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(Arrays.stream(modules).map(SwerveModule::getCurrentState)
                .toArray(SwerveModuleState[]::new));
    }

    public void toggleMode() {
        for (SwerveModule module : modules)
            module.toggleMode();
    }

    public void brake() {
        for (SwerveModule module : modules)
            module.brake();
    }

    public void coast() {
        for (SwerveModule module : modules)
            module.coast();
    }

    public double[][] getPIDConstants() {
        return new double[][] {
                xPIDController,
                yPIDController,
                thetaPIDController
        };
    }

}
