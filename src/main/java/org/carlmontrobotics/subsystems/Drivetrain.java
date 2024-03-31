package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Drivetrainc.*;

import java.util.Arrays;
import java.util.function.Supplier;

import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.commands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.commands.TeleopDrive;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SensorFactory;
import org.carlmontrobotics.lib199.swerve.SwerveModule;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2
    private Pose2d autoGyroOffset = new Pose2d(0., 0., new Rotation2d(0.));
    // ^used by PathPlanner for chaining paths

    private SwerveDriveKinematics kinematics = null;
    private SwerveDriveOdometry odometry = null;
    private Field2d field = new Field2d();

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

    public Drivetrain() {
        SmartDashboard.putNumber("set x", 0);
        SmartDashboard.putNumber("set y", 0);
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

            // for(CANSparkMax driveMotor : driveMotors)
            // driveMotor.setSmartCurrentLimit(80);

        }

        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(),
                new Pose2d());

        // Setup autopath builder
        configurePPLAutoBuilder();
        SmartDashboard.putNumber("chassis speeds x", 0);
                        SmartDashboard.putNumber("chassis speeds y", 0);

                                    SmartDashboard.putNumber("chassis speeds theta", 0);
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

    public void keepRotateMotorsAtDegrees(int angle) {
        for (SwerveModule module : modules) {
            module.turnPeriodic();
            module.move(0.0000000000001, angle);
        }
    }

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

        field.setRobotPose(odometry.getPoseMeters());


        odometry.update(gyro.getRotation2d(), getModulePositions());
        //odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        { 
            SmartDashboard.putNumber("front left encoder", moduleFL.getModuleAngle());
            SmartDashboard.putNumber("front right encoder", moduleFR.getModuleAngle());
            SmartDashboard.putNumber("back left encoder", moduleBL.getModuleAngle());
            SmartDashboard.putNumber("back right encoder", moduleBR.getModuleAngle());
       }
        SmartDashboard.putNumber("Odometry X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Odometry Y", getPose().getTranslation().getY());
        //setPose(new Pose2d(SmartDashboard.getNumber("set x", getPose().getTranslation().getX()), SmartDashboard.getNumber("set y", getPose().getTranslation().getY()), Rotation2d.fromDegrees(getHeading())));
        // SmartDashboard.putNumber("Odometry X", getPose().getTranslation().getX());
        // SmartDashboard.putNumber("Odometry Y", getPose().getTranslation().getY());
        // // // SmartDashboard.putNumber("Pitch", gyro.getPitch());
        // // // SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // // // SmartDashboard.putNumber("AdjRoll", gyro.getPitch() - initPitch);
        // // // SmartDashboard.putNumber("AdjPitch", gyro.getRoll() - initRoll);
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        // SmartDashboard.putNumber("Compass Offset", compassOffset);
        SmartDashboard.putBoolean("Current Magnetic Field Disturbance", gyro.isMagneticDisturbance());
        // SmartDashboard.putNumber("front left encoder", moduleFL.getModuleAngle());
        // SmartDashboard.putNumber("front right encoder", moduleFR.getModuleAngle());
        // SmartDashboard.putNumber("back left encoder", moduleBL.getModuleAngle());
        // SmartDashboard.putNumber("back right encoder", moduleBR.getModuleAngle());

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // for (SwerveModule module : modules)
        // SendableRegistry.addChild(this, module);

        // builder.addBooleanProperty("Magnetic Field Disturbance",
        // gyro::isMagneticDisturbance, null);
        // builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
        // builder.addBooleanProperty("Field Oriented", () -> fieldOriented,
        // fieldOriented -> this.fieldOriented = fieldOriented);
        // builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
        // builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
        // builder.addDoubleProperty("Odometry Heading", () ->
        // getPose().getRotation().getDegrees(), null);
        // builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
        // builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
        // builder.addDoubleProperty("Pitch", gyro::getPitch, null);
        // builder.addDoubleProperty("Roll", gyro::getRoll, null);
        // builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset ->
        // this.fieldOffset = fieldOffset);
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
            SmartDashboard.putNumber("moduleIn" + Integer.toString(i), moduleStates[i].angle.getDegrees());
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i],
                    Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            SmartDashboard.putNumber("moduleOT" + Integer.toString(i), moduleStates[i].angle.getDegrees());

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
            SmartDashboard.putNumber("chassis speeds x", cs.vxMetersPerSecond);
            SmartDashboard.putNumber("chassis speeds y", cs.vyMetersPerSecond);
            SmartDashboard.putNumber("chassis speeds theta", cs.omegaRadiansPerSecond);

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
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d initialPose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), initialPose);
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
