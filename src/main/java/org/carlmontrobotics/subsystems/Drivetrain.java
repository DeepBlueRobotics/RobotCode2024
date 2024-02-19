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
import edu.wpi.first.units.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
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
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.examples.rapidreactcommandbot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class Drivetrain extends SubsystemBase {
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2

    private SwerveDriveKinematics kinematics = null;
    private SwerveDriveOdometry odometry = null;
    private SwerveModule modules[];
    private boolean fieldOriented = true;
    private double fieldOffset = 0;
    //FIXME not for permanent use!!
    private CANSparkMax[] driveMotors;
    private CANSparkMax[] turnMotors = new CANSparkMax[] {null,null,null,null};
    // gyro
    public final float initPitch;
    public final float initRoll;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage>[] m_appliedVoltage = new MutableMeasure[8];
    
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance>[] m_distance = new MutableMeasure[4];
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>>[] m_velocity = new MutableMeasure[4];
    // edu.wpi.first.math.util.Units.Rotations beans;
    private final MutableMeasure<Angle>[] m_revs = new MutableMeasure[4];
    private final MutableMeasure<Velocity<Angle>>[] m_revs_vel = new MutableMeasure[4];

    private enum SysIdTest {
        FRONT_DRIVE,
        BACK_DRIVE,
        ALL_DRIVE,
        // FLBR_TURN,
        // FRBL_TURN,
        // ALL_TURN
        FL_ROT,
        FR_ROT,
        BL_ROT,
        BR_ROT
    }

    private SendableChooser<SysIdTest> sysIdChooser = new SendableChooser<>();

    //ROUTINES FOR SYSID
    private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(Volts.of(.1).per(Seconds.of(.1)), Volts.of(.6), Seconds.of(5));
    //DRIVE
    private void motorLogShort_drive(SysIdRoutineLog log, int id){
        String name = new String[] {"fl","fr","bl","br"}[id];
        log.motor(name)
            .voltage(m_appliedVoltage[id].mut_replace(
                    driveMotors[id].getBusVoltage() * driveMotors[id].getAppliedOutput(), Volts))
            .linearPosition(
                    m_distance[id].mut_replace(driveMotors[id].getEncoder().getPosition(), Meters))
            .linearVelocity(m_velocity[id].mut_replace(driveMotors[id].getEncoder().getVelocity(),
                    MetersPerSecond));
    }
    // Create a new SysId routine for characterizing the drive.
    private SysIdRoutine frontOnlyDriveRoutine = new SysIdRoutine(
        defaultSysIdConfig,
        new SysIdRoutine.Mechanism(
            // Tell SysId how to give the driving voltage to the motors.
            (Measure<Voltage> volts) -> {
                driveMotors[0].setVoltage(volts.in(Volts));
                driveMotors[1].setVoltage(volts.in(Volts));
                modules[2].coast();
                modules[3].coast();                                                     
            },
            log -> {// FRONT
                motorLogShort_drive(log,0);//fl named automatically
                motorLogShort_drive(log,1);//fr
            },
            this
        )
    );

    private SysIdRoutine backOnlyDriveRoutine = new SysIdRoutine(
        defaultSysIdConfig,
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                modules[0].coast();
                modules[1].coast();
                driveMotors[2].setVoltage(volts.in(Volts));
                driveMotors[3].setVoltage(volts.in(Volts));
            },
            log -> {// BACK
                motorLogShort_drive(log,2);//bl
                motorLogShort_drive(log,3);//br
            },
            this
        )
    );

    private SysIdRoutine allWheelsDriveRoutine = new SysIdRoutine(
        defaultSysIdConfig, 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                for (CANSparkMax dm : driveMotors) {
                    dm.setVoltage(volts.in(Volts));
                }
            }, 
            log -> {
                motorLogShort_drive(log,0);//fl named automatically
                motorLogShort_drive(log,1);//fr
                motorLogShort_drive(log,2);//bl
                motorLogShort_drive(log,3);//br
            }, 
            this
        )
    );
    private SysIdRoutine sysidroutshort_turn(int id, String logname){
        return new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(.1).per(Seconds.of(.1)), Volts.of(.6), Seconds.of(3)), 
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> turnMotors[id].setVoltage(volts.in(Volts)),
                log -> log.motor(logname+"_turn")
                    .voltage(m_appliedVoltage[id+4].mut_replace(
                            //^because drivemotors take up the first 4 slots of the unit holders
                            turnMotors[id].getBusVoltage() * turnMotors[id].getAppliedOutput(), Volts))
                    .angularPosition(m_revs[id].mut_replace(turnMotors[id].getEncoder().getPosition(), Degrees))
                    .angularVelocity(m_revs_vel[id].mut_replace(turnMotors[id].getEncoder().getVelocity(), DegreesPerSecond)),
                this
            )
        );
    }
    //as always, fl/fr/bl/br
    private SysIdRoutine[] rotateRoutine = new SysIdRoutine[] {
        sysidroutshort_turn(0,"fl"),//woaw, readable code???
        sysidroutshort_turn(1,"fr"),
        sysidroutshort_turn(2,"bl"),
        sysidroutshort_turn(3,"br")
    };

    private ShuffleboardTab sysIdTab = Shuffleboard.getTab("Drivetrain SysID");

    // void sysidtabshorthand(String name, SysIdRoutine.Direction dir, int width, int height){
    //     sysIdTab.add(name, dir).withSize(width, height);
    // }
    void sysidtabshorthand_qsi(String name, SysIdRoutine.Direction dir){
        sysIdTab.add(name, sysIdQuasistatic(dir)).withSize(2,1);
    }
    void sysidtabshorthand_dyn(String name, SysIdRoutine.Direction dir){
        sysIdTab.add(name, sysIdDynamic(dir)).withSize(2,1);
    }

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
                    turnMotors[0] = MotorControllerFactory.createSparkMax(turnFrontLeftPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortFL), 0,
                    pitchSupplier, rollSupplier);
            // Forward-Right
            moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR,
                    driveMotors[1] = MotorControllerFactory.createSparkMax(driveFrontRightPort, MotorConfig.NEO),
                    turnMotors[1] = MotorControllerFactory.createSparkMax(turnFrontRightPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortFR), 1,
                    pitchSupplier, rollSupplier);

            // Backward-Left
            moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL,
                    driveMotors[2] = MotorControllerFactory.createSparkMax(driveBackLeftPort, MotorConfig.NEO),
                    turnMotors[2] = MotorControllerFactory.createSparkMax(turnBackLeftPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortBL), 2,
                    pitchSupplier, rollSupplier);
            // Backward-Right
            moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR,
                    driveMotors[3] = MotorControllerFactory.createSparkMax(driveBackRightPort, MotorConfig.NEO),
                    turnMotors[3] = MotorControllerFactory.createSparkMax(turnBackRightPort, MotorConfig.NEO),
                    SensorFactory.createCANCoder(canCoderPortBR), 3,
                    pitchSupplier, rollSupplier);
            modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
            
            for (CANSparkMax driveMotor : driveMotors) {
                driveMotor.setOpenLoopRampRate(secsPer12Volts);
                driveMotor.getEncoder().setPositionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
                driveMotor.getEncoder().setVelocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing / 60);
            }
            for (CANSparkMax turnMotor : turnMotors) {
                turnMotor.getEncoder().setPositionConversionFactor(360 / turnGearing);
                turnMotor.getEncoder().setVelocityConversionFactor(360 / turnGearing / 60);
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
            
            
            sysidtabshorthand_qsi("Quasistatic Forward",SysIdRoutine.Direction.kForward);
            sysidtabshorthand_qsi("Quasistatic Backward",SysIdRoutine.Direction.kReverse);
            sysidtabshorthand_dyn("Dynamic Forward",SysIdRoutine.Direction.kForward);
            sysidtabshorthand_dyn("Dynamic Backward",SysIdRoutine.Direction.kReverse);

            sysIdChooser.addOption("Front Only Drive", SysIdTest.FRONT_DRIVE);
            sysIdChooser.addOption("Back Only Drive", SysIdTest.BACK_DRIVE);
            sysIdChooser.addOption("All Drive", SysIdTest.ALL_DRIVE);
            // sysIdChooser.addOption("fl-br Turn", SysIdTest.FLBR_TURN);
            // sysIdChooser.addOption("fr-bl Turn", SysIdTest.FRBL_TURN);
            // sysIdChooser.addOption("All Turn", SysIdTest.ALL_TURN);
            sysIdChooser.addOption("FL Rotate", SysIdTest.FL_ROT);
            sysIdChooser.addOption("FR Rotate", SysIdTest.FR_ROT);
            sysIdChooser.addOption("BL Rotate", SysIdTest.BL_ROT);
            sysIdChooser.addOption("BR Rotate", SysIdTest.BR_ROT);

            sysIdTab
                .add(sysIdChooser)
                .withSize(2, 1);

            sysIdTab.add("ALL THE SYSID TESTS", allTheSYSID())//is this legal??
                .withSize(2, 1);

            sysIdTab.add(this);

            for (int i = 0; i < 8; i++) {//first four are drive, next 4 are turn motors
                m_appliedVoltage[i] = mutable(Volts.of(0));
            }
            for (int i = 0; i < 4; i++) {
                m_distance[i] = mutable(Meters.of(0));
                m_velocity[i] = mutable(MetersPerSecond.of(0));

                m_revs[i] = mutable(Degrees.of(0));
                m_revs_vel[i] = mutable(DegreesPerSecond.of(0));
            }

            SmartDashboard.putNumber("Desired Angle", 0);
        }
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction, int frontorback) {
    //     switch(frontorback) {
    //         case 0:
    //             return frontOnlyRoutine.quasistatic(direction);
    //         case 1:
    //             return backOnlyRoutine.quasistatic(direction);
    //         case 2:
    //             return allWheelsRoutine.quasistatic(direction);
    //     }
    //     return new PrintCommand("Invalid Command");
    // }

    private SysIdTest selector() {
        SysIdTest test = sysIdChooser.getSelected();
        System.out.println("Test Selected: " + test);
        return test;
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SelectCommand<>(
            Map.ofEntries(
                //DRIVE
                Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running front only quasistatic forward") :
                        new PrintCommand("Running front only quasistatic backward"), 
                    frontOnlyDriveRoutine.quasistatic(direction)
                )),
                Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running back only quasistatic forward") :
                        new PrintCommand("Running back only quasistatic backward"), 
                    backOnlyDriveRoutine.quasistatic(direction)
                )),
                Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running all drive quasistatic forward") :
                        new PrintCommand("Running all drive quasistatic backward"), 
                    allWheelsDriveRoutine.quasistatic(direction)
                )),
                //ROTATE
                Map.entry(SysIdTest.FL_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running FL rotate quasistatic forward") :
                        new PrintCommand("Running FL rotate quasistatic backward"), 
                    rotateRoutine[0].quasistatic(direction)
                )),
                Map.entry(SysIdTest.FR_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running FR rotate quasistatic forward") :
                        new PrintCommand("Running FR rotate quasistatic backward"), 
                    rotateRoutine[1].quasistatic(direction)
                )),
                Map.entry(SysIdTest.BL_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running BL rotate quasistatic forward") :
                        new PrintCommand("Running BL rotate quasistatic backward"), 
                    rotateRoutine[2].quasistatic(direction)
                )),
                Map.entry(SysIdTest.BR_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running BR rotate quasistatic forward") :
                        new PrintCommand("Running BR rotate quasistatic backward"), 
                    rotateRoutine[3].quasistatic(direction)
                ))
                
                // //TURN
                // Map.entry(SysIdTest.FLBR_TURN, new ParallelCommandGroup(
                //     direction == SysIdRoutine.Direction.kForward ? 
                //         new PrintCommand("Running fL-bR turn quasistatic forward") :
                //         new PrintCommand("Running fL-bR turn quasistatic backward"), 
                //     flbrTurn.quasistatic(direction)
                // )),
                // Map.entry(SysIdTest.FRBL_TURN, new ParallelCommandGroup(
                //     direction == SysIdRoutine.Direction.kForward ? 
                //         new PrintCommand("Running fR-bL turn quasistatic forward") :
                //         new PrintCommand("Running fR-bL turn quasistatic backward"), 
                //     frblTurn.quasistatic(direction)
                // )),
                // Map.entry(SysIdTest.ALL_TURN, new ParallelCommandGroup(
                //     direction == SysIdRoutine.Direction.kForward ? 
                //         new PrintCommand("Running all turn quasistatic forward") :
                //         new PrintCommand("Running all turn quasistatic backward"), 
                //     allWheelsTurn.quasistatic(direction)
                // ))
            ),
            this::selector
        );
    }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction, int frontorback) {
    //     switch(frontorback) {
    //         case 0:
    //             return frontOnlyDrive.dynamic(direction);
    //         case 1:
    //             return backOnlyDrive.dynamic(direction);
    //         case 2:
    //             return allWheelsDrive.dynamic(direction);
    //     }
    //     return new PrintCommand("Invalid Command");
    // }
    private Command allTheSYSID(SysIdRoutine.Direction direction){
        return new SequentialCommandGroup(
            frontOnlyDriveRoutine.dynamic(direction),
            backOnlyDriveRoutine.dynamic(direction),
            allWheelsDriveRoutine.dynamic(direction),
            rotateRoutine[0].dynamic(direction),
            rotateRoutine[1].dynamic(direction),
            rotateRoutine[2].dynamic(direction),
            rotateRoutine[3].dynamic(direction),

            frontOnlyDriveRoutine.quasistatic(direction),
            backOnlyDriveRoutine.quasistatic(direction),
            allWheelsDriveRoutine.quasistatic(direction),
            rotateRoutine[0].quasistatic(direction),
            rotateRoutine[1].quasistatic(direction),
            rotateRoutine[2].quasistatic(direction),
            rotateRoutine[3].quasistatic(direction)
        );
    }
    public Command allTheSYSID(){
        return new SequentialCommandGroup(
            allTheSYSID(SysIdRoutine.Direction.kForward),
            allTheSYSID(SysIdRoutine.Direction.kReverse)
        );
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new SelectCommand<>(
            Map.ofEntries(
                //DRIVE
                Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running front only dynamic forward") :
                        new PrintCommand("Running front only dynamic backward"), 
                    frontOnlyDriveRoutine.dynamic(direction)
                )),
                Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running back only dynamic forward") :
                        new PrintCommand("Running back only dynamic backward"),
                    backOnlyDriveRoutine.dynamic(direction)
                )),
                Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running all wheels dynamic forward") :
                        new PrintCommand("Running all wheels dynamic backward"),
                    allWheelsDriveRoutine.dynamic(direction)
                )),
                //ROTATE
                Map.entry(SysIdTest.FL_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running FL rotate dynamic forward") :
                        new PrintCommand("Running FL rotate dynamic backward"), 
                    rotateRoutine[0].dynamic(direction)
                )),
                Map.entry(SysIdTest.FR_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running FR rotate dynamic forward") :
                        new PrintCommand("Running FR rotate dynamic backward"), 
                    rotateRoutine[1].dynamic(direction)
                )),
                Map.entry(SysIdTest.BL_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running BL rotate dynamic forward") :
                        new PrintCommand("Running BL rotate dynamic backward"), 
                    rotateRoutine[2].dynamic(direction)
                )),
                Map.entry(SysIdTest.BR_ROT, new ParallelCommandGroup(
                    direction == SysIdRoutine.Direction.kForward ? 
                        new PrintCommand("Running BR rotate dynamic forward") :
                        new PrintCommand("Running BR rotate dynamic backward"), 
                    rotateRoutine[3].dynamic(direction)
                ))
            ),
            this::selector
        );
    }

    @Override
    public void periodic() {
        // lobotomized to prevent ucontrollabe swerve behavior
        // FIXME: unlobotomize lib199
        moduleFL.periodic();
        // for (SwerveModule module : modules)
        //     module.periodic();
        double desiredGoal = SmartDashboard.getNumber("Desired Angle", 0);
        moduleFL.move(0, desiredGoal);

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
        SmartDashboard.putNumber("front left encoder", moduleFL.getModuleAngle());
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
