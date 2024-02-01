package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Drivetrain.*;

import java.util.Arrays;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SensorFactory;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.swerve.SwerveModule;
import org.carlmontrobotics.commands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.commands.TeleopDrive;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
   private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); // Also try kUSB and kUSB2

   private SwerveDriveKinematics kinematics = null;
   private SwerveDriveOdometry odometry = null;
   //private SwerveDrivePoseEstimator poseEstimator = null;
   private SwerveModule modules[];
   private boolean fieldOriented = true;
   private double fieldOffset = 0;
   //gyro
   public final float initPitch;
   public final float initRoll;

   public Drivetrain() {

       // Calibrate Gyro
       {
           double initTimestamp = Timer.getFPGATimestamp();
           double currentTimestamp = initTimestamp;
           while (gyro.isCalibrating() && currentTimestamp - initTimestamp < 10) {
               currentTimestamp = Timer.getFPGATimestamp();
               try {
                   Thread.sleep(1000);//1 second
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

           CANSparkMax[] driveMotors = new CANSparkMax[4];

           SwerveModule moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL,
                   driveMotors[0] = MotorControllerFactory.createSparkMax(driveFrontLeftPort, MotorConfig.NEO),
                   MotorControllerFactory.createSparkMax(turnFrontLeftPort, MotorConfig.NEO),
                   SensorFactory.createCANCoder(canCoderPortFL), 0,
                   pitchSupplier, rollSupplier);
           // Forward-Right
           SwerveModule moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR,
                   driveMotors[1] = MotorControllerFactory.createSparkMax(driveFrontRightPort, MotorConfig.NEO),
                   MotorControllerFactory.createSparkMax(turnFrontRightPort, MotorConfig.NEO),
                   SensorFactory.createCANCoder(canCoderPortFR), 1,
                   pitchSupplier, rollSupplier);
           // Backward-Left
           SwerveModule moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL,
                   driveMotors[2] = MotorControllerFactory.createSparkMax(driveBackLeftPort, MotorConfig.NEO),
                   MotorControllerFactory.createSparkMax(turnBackLeftPort, MotorConfig.NEO),
                   SensorFactory.createCANCoder(canCoderPortBL), 2,
                   pitchSupplier, rollSupplier);
           // Backward-Right
           SwerveModule moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR,
                   driveMotors[3] = MotorControllerFactory.createSparkMax(driveBackRightPort, MotorConfig.NEO),
                   MotorControllerFactory.createSparkMax(turnBackRightPort, MotorConfig.NEO),
                   SensorFactory.createCANCoder(canCoderPortBR), 3,
                   pitchSupplier, rollSupplier);
           modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
           for(CANSparkMax driveMotor: driveMotors) driveMotor.setOpenLoopRampRate(secsPer12Volts);
           //for(CANSparkMax driveMotor : driveMotors) driveMotor.setSmartCurrentLimit(80);
       }


       odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), new Pose2d());
       //poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), new Pose2d());
   }

   @Override
   public void periodic() {
       for (SwerveModule module : modules) module.periodic();

       // Update the odometry with current heading and encoder position
       odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

       autoCancelDtCommand();

       SmartDashboard.putNumber("Odometry X", getPose().getTranslation().getX());
       SmartDashboard.putNumber("Odometry Y", getPose().getTranslation().getY());;
       // SmartDashboard.putNumber("Pitch", gyro.getPitch());
       // SmartDashboard.putNumber("Roll", gyro.getRoll());
       SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
       SmartDashboard.putNumber("Robot Heading", getHeading());
       // SmartDashboard.putNumber("AdjRoll", gyro.getPitch() - initPitch);
       // SmartDashboard.putNumber("AdjPitch", gyro.getRoll() - initRoll);
       SmartDashboard.putBoolean("Field Oriented", fieldOriented);
       // SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
       // SmartDashboard.putNumber("Compass Offset", compassOffset);
       // SmartDashboard.putBoolean("Current Magnetic Field Disturbance",
       // gyro.isMagneticDisturbance());
   }

   public void autoCancelDtCommand() {
       if(!(getDefaultCommand() instanceof TeleopDrive) || DriverStation.isAutonomous()) return;

       // Use hasDriverInput to get around acceleration limiting on slowdown
       if(((TeleopDrive) getDefaultCommand()).hasDriverInput()) {
           Command currentDtCommand = getCurrentCommand();
           if(currentDtCommand != getDefaultCommand() && !(currentDtCommand instanceof RotateToFieldRelativeAngle) && currentDtCommand != null) {
               currentDtCommand.cancel();
           }
       }
   }

   @Override
   public void initSendable(SendableBuilder builder) {
       super.initSendable(builder);

       for(SwerveModule module : modules) SendableRegistry.addChild(this, module);

       builder.addBooleanProperty("Magnetic Field Disturbance", gyro::isMagneticDisturbance, null);
       builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
       builder.addBooleanProperty("Field Oriented", () -> fieldOriented, fieldOriented -> this.fieldOriented = fieldOriented);
       builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
       builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
       builder.addDoubleProperty("Odometry Heading", () -> getPose().getRotation().getDegrees(), null);
       builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
       builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
       builder.addDoubleProperty("Pitch", gyro::getPitch, null);
       builder.addDoubleProperty("Roll", gyro::getRoll, null);
       builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset -> this.fieldOffset = fieldOffset);
   }

   //#region Drive Methods

   /**
    * Drives the robot using the given x, y, and rotation speed
    *
    * @param forward  The desired forward speed, in m/s. Forward is positive.
    * @param strafe   The desired strafe speed, in m/s. Left is positive.
    * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive
    */
   public void drive(double forward, double strafe, double rotation) {
       drive(getSwerveStates(forward, strafe, rotation));
   }

   public void drive(SwerveModuleState[] moduleStates) {
       SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

       // Move the modules based on desired (normalized) speed, desired angle, max
       // speed, drive modifier, and whether or not to reverse turning.
       for (int i = 0; i < 4; i++) {
           moduleStates[i] = SwerveModuleState.optimize(moduleStates[i],
                   Rotation2d.fromDegrees(modules[i].getModuleAngle()));
           modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
       }
   }

   public void stop() {
       for(SwerveModule module: modules) module.move(0, 0);
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
    * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive.
    * @return A ChassisSpeeds object.
    */
   private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
       ChassisSpeeds speeds;
       if (fieldOriented) {
           speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()));
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
    * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive.
    * @return A SwerveModuleState array, one for each side of the drivetrain (FL,
    *         FR, etc.).
    */
   private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
       return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, rotation));
   }

   //#endregion

   //#region Getters and Setters

   // returns a value from -180 to 180
   public double getHeading() {
       double x = gyro.getAngle();
       if (fieldOriented) x -= fieldOffset;
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
       for (SwerveModule module: modules)
           module.toggleMode();
   }

   public void brake() {
       for (SwerveModule module: modules)
           module.brake();
   }

   public void coast() {
       for (SwerveModule module: modules)
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
