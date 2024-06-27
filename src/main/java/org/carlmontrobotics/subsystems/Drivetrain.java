package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Drivetrainc.*;
import static org.carlmontrobotics.Constants.Limelightc.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;

import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.commands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.commands.TeleopDrive;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SensorFactory;
import org.carlmontrobotics.lib199.swerve.SwerveModule;
import static org.carlmontrobotics.Config.CONFIG;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import org.carlmontrobotics.lib199.swerve.SwerveModuleSim;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
public class Drivetrain extends SubsystemBase {
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2
    private Pose2d autoGyroOffset = new Pose2d(0., 0., new Rotation2d(0.));
    // ^used by PathPlanner for chaining paths

    private SwerveDriveKinematics kinematics = null;
    // private SwerveDriveOdometry odometry = null;

    private SwerveDrivePoseEstimator poseEstimator = null;

    private SwerveModule modules[];
    private boolean fieldOriented = true;
    private double fieldOffset = 0;
    // FIXME not for permanent use!!
    private CANSparkMax[] driveMotors = new CANSparkMax[] { null, null, null, null };
    private CANSparkMax[] turnMotors = new CANSparkMax[] { null, null, null, null };
    private CANcoder[] turnEncoders = new CANcoder[] { null, null, null, null };

    // gyro
    public final float initPitch;
    public final float initRoll;

    // debug purposes
    private SwerveModule moduleFL;
    private SwerveModule moduleFR;
    private SwerveModule moduleBL;
    private SwerveModule moduleBR;

    private final Field2d field = new Field2d();

    private SwerveModuleSim[] moduleSims;
    private SimDouble gyroYawSim;
    private Timer simTimer = new Timer();

    public Drivetrain() {
        SmartDashboard.putNumber("Pose Estimator set x", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Pose Estimator set y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Pose Estimator set rotation", getPose().getRotation().getDegrees());
        setPose(new Pose2d(SmartDashboard.getNumber("Pose Estimator set x", getPose().getTranslation().getX()),
                SmartDashboard.getNumber("Pose Estimator set y", getPose().getTranslation().getY()),
                Rotation2d.fromDegrees(SmartDashboard.getNumber("Pose Estimator set rotation",
                        getPose().getRotation().getDegrees()))));

        SmartDashboard.putNumber("pose estimator std dev x", STD_DEV_X_METERS);
        SmartDashboard.putNumber("pose estimator std dev y", STD_DEV_Y_METERS);

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
            // this.resetFieldOrientation();
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

            moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL,
                    driveMotors[0] = MotorControllerFactory.createSparkMax(driveFrontLeftPort, MotorConfig.NEO),
                    turnMotors[0] = MotorControllerFactory.createSparkMax(turnFrontLeftPort, MotorConfig.NEO),
                    turnEncoders[0] = SensorFactory.createCANCoder(canCoderPortFL), 0,
                    pitchSupplier, rollSupplier);
            // Forward-Right
            moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR,
                    driveMotors[1] = MotorControllerFactory.createSparkMax(driveFrontRightPort, MotorConfig.NEO),
                    turnMotors[1] = MotorControllerFactory.createSparkMax(turnFrontRightPort, MotorConfig.NEO),
                    turnEncoders[1] = SensorFactory.createCANCoder(canCoderPortFR), 1,
                    pitchSupplier, rollSupplier);

            // Backward-Left
            moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL,
                    driveMotors[2] = MotorControllerFactory.createSparkMax(driveBackLeftPort, MotorConfig.NEO),
                    turnMotors[2] = MotorControllerFactory.createSparkMax(turnBackLeftPort, MotorConfig.NEO),
                    turnEncoders[2] = SensorFactory.createCANCoder(canCoderPortBL), 2,
                    pitchSupplier, rollSupplier);
            // Backward-Right
            moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR,
                    driveMotors[3] = MotorControllerFactory.createSparkMax(driveBackRightPort, MotorConfig.NEO),
                    turnMotors[3] = MotorControllerFactory.createSparkMax(turnBackRightPort, MotorConfig.NEO),
                    turnEncoders[3] = SensorFactory.createCANCoder(canCoderPortBR), 3,
                    pitchSupplier, rollSupplier);
            modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };

            if (RobotBase.isSimulation()) {
                moduleSims = new SwerveModuleSim[] {
                    moduleFL.createSim(), moduleFR.createSim(), moduleBL.createSim(), moduleBR.createSim()
                };
                gyroYawSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");
            }

            for (CANSparkMax driveMotor : driveMotors) {
                driveMotor.setOpenLoopRampRate(secsPer12Volts);
                driveMotor.getEncoder().setPositionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
                driveMotor.getEncoder().setVelocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing / 60);
                driveMotor.getEncoder().setAverageDepth(2);
                driveMotor.getEncoder().setMeasurementPeriod(16);
                driveMotor.setSmartCurrentLimit(MotorConfig.NEO.currentLimitAmps);
            }
            for (CANSparkMax turnMotor : turnMotors) {
                turnMotor.getEncoder().setPositionConversionFactor(360 / turnGearing);
                turnMotor.getEncoder().setVelocityConversionFactor(360 / turnGearing / 60);
                turnMotor.getEncoder().setAverageDepth(2);
                turnMotor.getEncoder().setMeasurementPeriod(16);
            }
            for (CANcoder coder : turnEncoders) {
                coder.getAbsolutePosition().setUpdateFrequency(500);
                coder.getPosition().setUpdateFrequency(500);
                coder.getVelocity().setUpdateFrequency(500);

            }

            SmartDashboard.putData("Field", field);
            SmartDashboard.putData(this);

            // for(CANSparkMax driveMotor : driveMotors)
            // driveMotor.setSmartCurrentLimit(80);

            // Must call this method for SysId to run
            if (CONFIG.isSysIdTesting()) {
                sysIdSetup();
            }
        }

        // odometry = new SwerveDriveOdometry(kinematics,
        // Rotation2d.fromDegrees(getHeading()), getModulePositions(),
        // new Pose2d());

        poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(),
                Rotation2d.fromDegrees(getHeading()),
                getModulePositions(),
                new Pose2d());

        // Setup autopath builder
        configurePPLAutoBuilder();
        // SmartDashboard.putNumber("chassis speeds x", 0);
        //                 SmartDashboard.putNumber("chassis speeds y", 0);

        //                             SmartDashboard.putNumber("chassis speeds theta", 0);
    }

    @Override
    public void simulationPeriodic() {
        for (var moduleSim : moduleSims) {
            moduleSim.update();
        }
        SwerveModuleState[] measuredStates =
            new SwerveModuleState[] {
                moduleFL.getCurrentState(), moduleFR.getCurrentState(), moduleBL.getCurrentState(), moduleBR.getCurrentState()
            };
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(measuredStates);

        double dtSecs = simTimer.get();
        simTimer.restart();

        Pose2d simPose = field.getRobotPose();
        simPose = simPose.exp(
                new Twist2d(
                    speeds.vxMetersPerSecond * dtSecs,
                    speeds.vyMetersPerSecond * dtSecs,
                    speeds.omegaRadiansPerSecond * dtSecs));
        double newAngleDeg = simPose.getRotation().getDegrees();
        // Subtract the offset computed the last time setPose() was called because odometry.update() adds it back.
        newAngleDeg -= simGyroOffset.getDegrees();
        newAngleDeg *= (isGyroReversed ? -1.0 : 1.0);
        gyroYawSim.set(newAngleDeg);
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction, int
    // frontorback) {
    // switch(frontorback) {
    // case 0:
    // return frontOnlyRoutine.quasistatic(direction);
    // case 1:
    // return backOnlyRoutine.quasistatic(direction);
    // case 2:
    // return allWheelsRoutine.quasistatic(direction);
    // }
    // return new PrintCommand("Invalid Command");
    // }

    @Override
    public void periodic() {
        // for (CANcoder coder : turnEncoders) {
        // SignalLogger.writeDouble("Regular position " + coder.toString(),
        // coder.getPosition().getValue());
        // SignalLogger.writeDouble("Velocity " + coder.toString(),
        // coder.getVelocity().getValue());
        // SignalLogger.writeDouble("Absolute position " + coder.toString(),
        // coder.getAbsolutePosition().getValue());
        // }
        // lobotomized to prevent ucontrollabe swerve behavior
        // turnMotors[2].setVoltage(SmartDashboard.getNumber("kS", 0));
        // moduleFL.periodic();
        // moduleFR.periodic();
        // moduleBL.periodic();
        // moduleBR.periodic();
        // double goal = SmartDashboard.getNumber("bigoal", 0);
        for (SwerveModule module : modules) {
            module.periodic();
            // module.move(0, goal);
        }

        // field.setRobotPose(odometry.getPoseMeters());

        field.setRobotPose(poseEstimator.getEstimatedPosition());

        // odometry.update(gyro.getRotation2d(), getModulePositions());

        poseEstimator.update(gyro.getRotation2d(), getModulePositions());
        //odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        updateMT2PoseEstimator();

        // // // SmartDashboard.putNumber("Pitch", gyro.getPitch());
        // // // SmartDashboard.putNumber("Roll", gyro.getRoll());
        // SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // // // SmartDashboard.putNumber("AdjRoll", gyro.getPitch() - initPitch);
        // // // SmartDashboard.putNumber("AdjPitch", gyro.getRoll() - initRoll);
        // SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        // SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        // SmartDashboard.putNumber("Compass Offset", compassOffset);
        // SmartDashboard.putBoolean("Current Magnetic Field Disturbance", gyro.isMagneticDisturbance());
        // SmartDashboard.putNumber("front left encoder", moduleFL.getModuleAngle());
        // SmartDashboard.putNumber("front right encoder", moduleFR.getModuleAngle());
        // SmartDashboard.putNumber("back left encoder", moduleBL.getModuleAngle());
        // SmartDashboard.putNumber("back right encoder", moduleBR.getModuleAngle());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        for (SwerveModule module : modules)
            SendableRegistry.addChild(this, module);

        builder.addBooleanProperty("Magnetic Field Disturbance",
        gyro::isMagneticDisturbance, null);
        builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
        builder.addBooleanProperty("Field Oriented", () -> fieldOriented,
        fieldOriented -> this.fieldOriented = fieldOriented);
        builder.addDoubleProperty("Pose Estimator set x", () -> getPose().getX(), null);
        builder.addDoubleProperty("Pose Estimator set y", () -> getPose().getY(), null);
        builder.addDoubleProperty("Pose Estimator set rotation", () ->
        getPose().getRotation().getDegrees(), null);
        builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
        builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
        builder.addDoubleProperty("Pitch", gyro::getPitch, null);
        builder.addDoubleProperty("Roll", gyro::getRoll, null);
        builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset ->
        this.fieldOffset = fieldOffset);
        builder.addDoubleProperty("FL Turn Encoder (Deg)",
                () -> moduleFL.getModuleAngle(), null);
        builder.addDoubleProperty("FR Turn Encoder (Deg)",
                () -> moduleFR.getModuleAngle(), null);
        builder.addDoubleProperty("BL Turn Encoder (Deg)",
                () -> moduleBL.getModuleAngle(), null);
        builder.addDoubleProperty("BR Turn Encoder (Deg)",
                () -> moduleBR.getModuleAngle(), null);

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
        for (int i = 0; i < 4; i++) {
            // SmartDashboard.putNumber("moduleIn" + Integer.toString(i), moduleStates[i].angle.getDegrees());
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i],
                    Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            // SmartDashboard.putNumber("moduleOT" + Integer.toString(i), moduleStates[i].angle.getDegrees());

            modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }
    
   public void configurePPLAutoBuilder() {
    /**
     * PATHPLANNER SETTINGS
     * Robot Width (m): .91
     * Robot Length(m): .94
     * Max Module Spd (m/s): 4.30
     * Default Constraints
     * Max Vel: 1.54, Max Accel: 6.86
     * Max Angvel: 360, Max AngAccel: 180 (guesses!)
     */
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getSpeeds,
        (ChassisSpeeds cs) -> {
            //cs.vxMetersPerSecond = -cs.vxMetersPerSecond;
            // SmartDashboard.putNumber("chassis speeds x", cs.vxMetersPerSecond);
            // SmartDashboard.putNumber("chassis speeds y", cs.vyMetersPerSecond);
            // SmartDashboard.putNumber("chassis speeds theta", cs.omegaRadiansPerSecond);

            drive(kinematics.toSwerveModuleStates(cs));  
        },
        new HolonomicPathFollowerConfig(
        new PIDConstants(xPIDController[0], xPIDController[1], xPIDController[2], 0), //translation (drive) pid vals
        new PIDConstants(thetaPIDController[0], thetaPIDController[1], thetaPIDController[2], 0), //rotation pid vals
        maxSpeed,
        swerveRadius,
        Autoc.replanningConfig,
        Robot.robot.getPeriod()//robot period
    ),
    () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red;
        //else:
        return false;
      },
      this
    );

    /*
     AutoBuilder.configureHolonomic(
    () -> getPose().plus(new Transform2d(autoGyroOffset.getTranslation(),autoGyroOffset.getRotation())),//position supplier
    (Pose2d pose) -> { autoGyroOffset=pose.times(-1); }, //position reset (by subtracting current pos)
    this::getSpeeds, //chassisSpeed supplier
    (ChassisSpeeds cs) -> drive(
            cs.vxMetersPerSecond, 
            -cs.vyMetersPerSecond,
            //flipped because drive assumes up is negative, but PPlanner assumes up is positive
            cs.omegaRadiansPerSecond
    ),
    new HolonomicPathFollowerConfig(
        new PIDConstants(drivekP[0], drivekI[0], drivekD[0], driveIzone), //translation (drive) pid vals
        new PIDConstants(turnkP_avg, 0., 0., turnIzone), //rotation pid vals
        maxSpeed,
        swerveRadius,
        Autoc.replanningConfig,
        Robot.robot.getPeriod()//robot period
    ),
    () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red;
        //else:
        return false;
      },
      this
    );
    */
   }

   public void autoCancelDtCommand() {
       if(!(getDefaultCommand() instanceof TeleopDrive) || DriverStation.isAutonomous()) return;

        // Use hasDriverInput to get around acceleration limiting on slowdown
        if (((TeleopDrive) getDefaultCommand()).hasDriverInput()) {
            Command currentDtCommand = getCurrentCommand();
            if (currentDtCommand != getDefaultCommand() && !(currentDtCommand instanceof RotateToFieldRelativeAngle)
                    && currentDtCommand != null) {
                currentDtCommand.cancel();
            }
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
        return getHeading();//...wait.
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getCurrentPosition).toArray(SwerveModulePosition[]::new);
    }

    public Pose2d getPose() {
        // return odometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    private Rotation2d simGyroOffset = new Rotation2d();
    public void setPose(Pose2d initialPose) {
        Rotation2d gyroRotation = gyro.getRotation2d();
        // odometry.resetPosition(gyroRotation, getModulePositions(), initialPose);

        poseEstimator.resetPosition(gyroRotation, getModulePositions(), initialPose);
        // Remember the offset that the above call to resetPosition() will cause the odometry.update() will add to the gyro rotation in the future
        // We need the offset so that we can compensate for it during simulationPeriodic().
        simGyroOffset = initialPose.getRotation().minus(gyroRotation);
        //odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), initialPose);
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

    public void resetPoseEstimator() {
        // odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());

        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
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

    // #region SysId Code

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

    // ROUTINES FOR SYSID
    // private SysIdRoutine.Config defaultSysIdConfig = new
    // SysIdRoutine.Config(Volts.of(.1).per(Seconds.of(.1)), Volts.of(.6),
    // Seconds.of(5));
    private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)),
            Volts.of(2.891), Seconds.of(10));

    // DRIVE
    private void motorLogShort_drive(SysIdRoutineLog log, int id) {
        String name = new String[] { "fl", "fr", "bl", "br" }[id];
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
                        motorLogShort_drive(log, 0);// fl named automatically
                        motorLogShort_drive(log, 1);// fr
                    },
                    this));

    private SysIdRoutine backOnlyDriveRoutine = new SysIdRoutine(
            defaultSysIdConfig,
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        modules[0].coast();
                        modules[1].coast();
                        modules[2].brake();
                        modules[3].brake();
                        driveMotors[2].setVoltage(volts.in(Volts));
                        driveMotors[3].setVoltage(volts.in(Volts));
                    },
                    log -> {// BACK
                        motorLogShort_drive(log, 2);// bl
                        motorLogShort_drive(log, 3);// br
                    },
                    this));

    private SysIdRoutine allWheelsDriveRoutine = new SysIdRoutine(
            defaultSysIdConfig,
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        for (CANSparkMax dm : driveMotors) {
                            dm.setVoltage(volts.in(Volts));
                        }
                    },
                    log -> {
                        motorLogShort_drive(log, 0);// fl named automatically
                        motorLogShort_drive(log, 1);// fr
                        motorLogShort_drive(log, 2);// bl
                        motorLogShort_drive(log, 3);// br
                    },
                    this));

    private SysIdRoutine sysidroutshort_turn(int id, String logname) {
        return new SysIdRoutine(
                defaultSysIdConfig,
                // new SysIdRoutine.Config(Volts.of(.1).per(Seconds.of(.1)), Volts.of(.6),
                // Seconds.of(3)),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> turnMotors[id].setVoltage(volts.in(Volts)),
                        log -> log.motor(logname + "_turn")
                                .voltage(m_appliedVoltage[id + 4].mut_replace(
                                        // ^because drivemotors take up the first 4 slots of the unit holders
                                        turnMotors[id].getBusVoltage() * turnMotors[id].getAppliedOutput(), Volts))
                                .angularPosition(
                                        m_revs[id].mut_replace(turnEncoders[id].getPosition().getValue(), Rotations))
                                .angularVelocity(m_revs_vel[id].mut_replace(
                                        turnEncoders[id].getVelocity().getValueAsDouble(), RotationsPerSecond)),
                        this));
    }

    // as always, fl/fr/bl/br
    private SysIdRoutine[] rotateRoutine = new SysIdRoutine[] {
            sysidroutshort_turn(0, "fl"), // woaw, readable code???
            sysidroutshort_turn(1, "fr"),
            sysidroutshort_turn(2, "bl"),
            sysidroutshort_turn(3, "br")
    };

    private ShuffleboardTab sysIdTab = Shuffleboard.getTab("Drivetrain SysID");

    // void sysidtabshorthand(String name, SysIdRoutine.Direction dir, int width,
    // int height){
    // sysIdTab.add(name, dir).withSize(width, height);
    // }
    void sysidtabshorthand_qsi(String name, SysIdRoutine.Direction dir) {
        sysIdTab.add(name, sysIdQuasistatic(dir)).withSize(2, 1);
    }

    void sysidtabshorthand_dyn(String name, SysIdRoutine.Direction dir) {
        sysIdTab.add(name, sysIdDynamic(dir)).withSize(2, 1);
    }

    private void sysIdSetup() {
        // SysId Setup
        {
            Supplier<SequentialCommandGroup> stopNwait = () -> new SequentialCommandGroup(
                    new InstantCommand(this::stop), new WaitCommand(2));

            /*
             * Alex's old sysId tests
             * sysIdTab.add("All sysid tests", new SequentialCommandGroup(
             * new
             * SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kForward,2),
             * (Command)stopNwait.get()),
             * new
             * SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kReverse,2),
             * (Command)stopNwait.get()),
             * new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kForward,2),
             * (Command)stopNwait.get()),
             * new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kReverse,2),
             * (Command)stopNwait.get())
             * ));
             * sysIdTab.add("All sysid tests - FRONT wheels", new SequentialCommandGroup(
             * new
             * SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kForward,0),
             * (Command)stopNwait.get()),
             * new
             * SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kReverse,0),
             * (Command)stopNwait.get()),
             * new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kForward,0),
             * (Command)stopNwait.get()),
             * new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kReverse,0),
             * (Command)stopNwait.get())
             * ));
             * sysIdTab.add("All sysid tests - BACK wheels", new SequentialCommandGroup(
             * new
             * SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kForward,1),
             * (Command)stopNwait.get()),
             * new
             * SequentialCommandGroup(sysIdQuasistatic(SysIdRoutine.Direction.kReverse,1),
             * (Command)stopNwait.get()),
             * new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kForward,1),
             * (Command)stopNwait.get()),
             * new SequentialCommandGroup(sysIdDynamic(SysIdRoutine.Direction.kReverse,1),
             * (Command)stopNwait.get())
             * ));
             */

            sysidtabshorthand_qsi("Quasistatic Forward", SysIdRoutine.Direction.kForward);
            sysidtabshorthand_qsi("Quasistatic Backward", SysIdRoutine.Direction.kReverse);
            sysidtabshorthand_dyn("Dynamic Forward", SysIdRoutine.Direction.kForward);
            sysidtabshorthand_dyn("Dynamic Backward", SysIdRoutine.Direction.kReverse);

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

            sysIdTab.add("ALL THE SYSID TESTS", allTheSYSID())// is this legal??
                    .withSize(2, 1);

            sysIdTab.add(this);

            for (int i = 0; i < 8; i++) {// first four are drive, next 4 are turn motors
                m_appliedVoltage[i] = mutable(Volts.of(0));
            }
            for (int i = 0; i < 4; i++) {
                m_distance[i] = mutable(Meters.of(0));
                m_velocity[i] = mutable(MetersPerSecond.of(0));

                m_revs[i] = mutable(Rotations.of(0));
                m_revs_vel[i] = mutable(RotationsPerSecond.of(0));
            }

            SmartDashboard.putNumber("Desired Angle", 0);

            SmartDashboard.putNumber("kS", 0);
        }
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction, int
    // frontorback) {
    // switch(frontorback) {
    // case 0:
    // return frontOnlyRoutine.quasistatic(direction);
    // case 1:
    // return backOnlyRoutine.quasistatic(direction);
    // case 2:
    // return allWheelsRoutine.quasistatic(direction);
    // }
    // return new PrintCommand("Invalid Command");
    // }

    private SysIdTest selector() {
        SysIdTest test = sysIdChooser.getSelected();
        System.out.println("Test Selected: " + test);
        return test;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SelectCommand<>(
                Map.ofEntries(
                        // DRIVE
                        Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running front only quasistatic forward")
                                        : new PrintCommand("Running front only quasistatic backward"),
                                frontOnlyDriveRoutine.quasistatic(direction))),
                        Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running back only quasistatic forward")
                                        : new PrintCommand("Running back only quasistatic backward"),
                                backOnlyDriveRoutine.quasistatic(direction))),
                        Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running all drive quasistatic forward")
                                        : new PrintCommand("Running all drive quasistatic backward"),
                                allWheelsDriveRoutine.quasistatic(direction))),
                        // ROTATE
                        Map.entry(SysIdTest.FL_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running FL rotate quasistatic forward")
                                        : new PrintCommand("Running FL rotate quasistatic backward"),
                                rotateRoutine[0].quasistatic(direction))),
                        Map.entry(SysIdTest.FR_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running FR rotate quasistatic forward")
                                        : new PrintCommand("Running FR rotate quasistatic backward"),
                                rotateRoutine[1].quasistatic(direction))),
                        Map.entry(SysIdTest.BL_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running BL rotate quasistatic forward")
                                        : new PrintCommand("Running BL rotate quasistatic backward"),
                                rotateRoutine[2].quasistatic(direction))),
                        Map.entry(SysIdTest.BR_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running BR rotate quasistatic forward")
                                        : new PrintCommand("Running BR rotate quasistatic backward"),
                                rotateRoutine[3].quasistatic(direction)))

                // //TURN
                // Map.entry(SysIdTest.FLBR_TURN, new ParallelCommandGroup(
                // direction == SysIdRoutine.Direction.kForward ?
                // new PrintCommand("Running fL-bR turn quasistatic forward") :
                // new PrintCommand("Running fL-bR turn quasistatic backward"),
                // flbrTurn.quasistatic(direction)
                // )),
                // Map.entry(SysIdTest.FRBL_TURN, new ParallelCommandGroup(
                // direction == SysIdRoutine.Direction.kForward ?
                // new PrintCommand("Running fR-bL turn quasistatic forward") :
                // new PrintCommand("Running fR-bL turn quasistatic backward"),
                // frblTurn.quasistatic(direction)
                // )),
                // Map.entry(SysIdTest.ALL_TURN, new ParallelCommandGroup(
                // direction == SysIdRoutine.Direction.kForward ?
                // new PrintCommand("Running all turn quasistatic forward") :
                // new PrintCommand("Running all turn quasistatic backward"),
                // allWheelsTurn.quasistatic(direction)
                // ))
                ),
                this::selector);
    }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction, int
    // frontorback) {
    // switch(frontorback) {
    // case 0:
    // return frontOnlyDrive.dynamic(direction);
    // case 1:
    // return backOnlyDrive.dynamic(direction);
    // case 2:
    // return allWheelsDrive.dynamic(direction);
    // }
    // return new PrintCommand("Invalid Command");
    // }
    private Command allTheSYSID(SysIdRoutine.Direction direction) {
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
                rotateRoutine[3].quasistatic(direction));
    }

    public Command allTheSYSID() {
        return new SequentialCommandGroup(
                allTheSYSID(SysIdRoutine.Direction.kForward),
                allTheSYSID(SysIdRoutine.Direction.kReverse));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new SelectCommand<>(
                Map.ofEntries(
                        // DRIVE
                        Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running front only dynamic forward")
                                        : new PrintCommand("Running front only dynamic backward"),
                                frontOnlyDriveRoutine.dynamic(direction))),
                        Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running back only dynamic forward")
                                        : new PrintCommand("Running back only dynamic backward"),
                                backOnlyDriveRoutine.dynamic(direction))),
                        Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running all wheels dynamic forward")
                                        : new PrintCommand("Running all wheels dynamic backward"),
                                allWheelsDriveRoutine.dynamic(direction))),
                        // ROTATE
                        Map.entry(SysIdTest.FL_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running FL rotate dynamic forward")
                                        : new PrintCommand("Running FL rotate dynamic backward"),
                                rotateRoutine[0].dynamic(direction))),
                        Map.entry(SysIdTest.FR_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running FR rotate dynamic forward")
                                        : new PrintCommand("Running FR rotate dynamic backward"),
                                rotateRoutine[1].dynamic(direction))),
                        Map.entry(SysIdTest.BL_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running BL rotate dynamic forward")
                                        : new PrintCommand("Running BL rotate dynamic backward"),
                                rotateRoutine[2].dynamic(direction))),
                        Map.entry(SysIdTest.BR_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running BR rotate dynamic forward")
                                        : new PrintCommand("Running BR rotate dynamic backward"),
                                rotateRoutine[3].dynamic(direction)))),
                this::selector);
    }

    public void keepRotateMotorsAtDegrees(int angle) {
        for (SwerveModule module : modules) {
            module.turnPeriodic();
            module.move(0.0000000000001, angle);
        }
    }

    // pose estimator stuff

    public void updateMT2PoseEstimator() {
        boolean rejectVisionUpdate = false;

        LimelightHelpers.SetRobotOrientation(SHOOTER_LL_NAME,
                poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate visionPoseEstimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(SHOOTER_LL_NAME);

        if (Math.abs(getGyroRate()) > MAX_TRUSTED_ANG_VEL_DEG_PER_SEC) { // degrees per second
            rejectVisionUpdate = true;
        }

        if (visionPoseEstimate.tagCount == 0) {
            rejectVisionUpdate = true;
        }

        if (!rejectVisionUpdate) {
            poseEstimator
                    .setVisionMeasurementStdDevs(
                            VecBuilder.fill(SmartDashboard.getNumber("pose estimator std dev x", STD_DEV_X_METERS),
                                    SmartDashboard.getNumber("pose estimator std dev y", STD_DEV_Y_METERS),
                                    STD_DEV_HEADING_RADS));
            poseEstimator.addVisionMeasurement(visionPoseEstimate.pose, visionPoseEstimate.timestampSeconds);
        }
    }

    public double getGyroRate() {
        return gyro.getRate();
    }
    // #endregion
}
