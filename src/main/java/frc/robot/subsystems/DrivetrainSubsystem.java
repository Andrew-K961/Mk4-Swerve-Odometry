// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is
	 * useful during initial testing of the robot.
	 */
	public static final double MAX_VOLTAGE = 8.0;
	// Measure the drivetrain's maximum velocity or calculate the theoretical.
	// The formula for calculating the theoretical maximum velocity is:
	// <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
	// pi
	// By default this value is setup for a Mk3 standard module using Falcon500s to
	// drive.
	// An example of this constant for a Mk4 L2 module with NEOs to drive is:
	// 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
	// SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
	/**
	 * The maximum velocity of the robot in meters per second.
	 * <p>
	 * This is a measure of how fast the robot should be able to drive in a straight
	 * line.
	 */
	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
			SdsModuleConfigurations.MK4_L2.getDriveReduction() *
			SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * <p>
	 * This is a measure of how fast the robot can rotate in place.
	 */
	// Here we calculate the theoretical maximum angular velocity. You can also
	// replace this with a measured amount.
	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

	// By default we use a Pigeon for our gyroscope. But if you use another
	// gyroscope, like a NavX, you can change this.
	// The important thing about how you configure your gyroscope is that rotating
	// the robot counter-clockwise should
	// cause the angle reading to increase until it wraps back over to zero.
	// Remove if you are using a Pigeon
	private final PigeonIMU pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
	// Uncomment if you are using a NavX
	private final AHRS navx = new AHRS(SerialPort.Port.kMXP/* , (byte) 200 */); // NavX connected over MXP

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	private final SwerveDriveOdometry odometry;
	private Pose2d calculatedPose;
	private Pose2d gyroPose;
	private ShuffleboardTab odometryTab;
	private final NetworkTableEntry estimatedX;
	private final NetworkTableEntry estimatedY;
	private final NetworkTableEntry rotation;
	private final NetworkTableEntry NavX;
	private final NetworkTableEntry NavY;

	public DrivetrainSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
		odometryTab = Shuffleboard.getTab("Odometry");

		estimatedX = odometryTab.add("Estimated X", 0).getEntry();
		estimatedY = odometryTab.add("Estimated Y", 0).getEntry();
		rotation = odometryTab.add("\"Estimated\" Rotation", 0).getEntry();
		NavX = odometryTab.add("NavX X", 0).getEntry();
		NavY = odometryTab.add("NavX Y", 0).getEntry();
		

		// There are 4 methods you can call to create your swerve modules.
		// The method you use depends on what motors you are using.
		//
		// Mk3SwerveModuleHelper.createFalcon500(...)
		// Your module has two Falcon 500s on it. One for steering and one for driving.
		//
		// Mk3SwerveModuleHelper.createNeo(...)
		// Your module has two NEOs on it. One for steering and one for driving.
		//
		// Mk3SwerveModuleHelper.createFalcon500Neo(...)
		// Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
		// and the NEO is for steering.
		//
		// Mk3SwerveModuleHelper.createNeoFalcon500(...)
		// Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
		// Falcon 500 is for steering.
		//
		// Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
		// class.

		// By default we will use Falcon 500s in standard configuration. But if you use
		// a different configuration or motors
		// you MUST change it. If you do not, your code will crash on startup.
		//  Setup motor configuration
		m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				// This can either be STANDARD or FAST depending on your gear configuration
				Mk4SwerveModuleHelper.GearRatio.L2,
				// This is the ID of the drive motor
				FRONT_LEFT_MODULE_DRIVE_MOTOR,
				// This is the ID of the steer motor
				FRONT_LEFT_MODULE_STEER_MOTOR,
				// This is the ID of the steer encoder
				FRONT_LEFT_MODULE_STEER_ENCODER,
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				FRONT_LEFT_MODULE_STEER_OFFSET);

		// We will do the same for the other modules
		m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				FRONT_RIGHT_MODULE_STEER_MOTOR,
				FRONT_RIGHT_MODULE_STEER_ENCODER,
				FRONT_RIGHT_MODULE_STEER_OFFSET);

		m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				BACK_LEFT_MODULE_DRIVE_MOTOR,
				BACK_LEFT_MODULE_STEER_MOTOR,
				BACK_LEFT_MODULE_STEER_ENCODER,
				BACK_LEFT_MODULE_STEER_OFFSET);

		m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(6, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				BACK_RIGHT_MODULE_DRIVE_MOTOR,
				BACK_RIGHT_MODULE_STEER_MOTOR,
				BACK_RIGHT_MODULE_STEER_ENCODER,
				BACK_RIGHT_MODULE_STEER_OFFSET);

		odometry = new SwerveDriveOdometry(m_kinematics, navx.getRotation2d());
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		// Remove if you are using a Pigeon
		pigeon.setFusedHeading(0.0);
		
		// Uncomment if you are using a NavX
		navx.zeroYaw();
		navx.resetDisplacement();
		calculatedPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
		gyroPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
		odometry.resetPosition(calculatedPose, navx.getRotation2d());
	}

	public Rotation2d getPigeonRotation() {
		// Remove if you are using a Pigeon
		return Rotation2d.fromDegrees(pigeon.getFusedHeading());
/*
		// Uncomment if you are using a NavX
		if (navx.isMagnetometerCalibrated()) {
			// // We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(navx.getFusedHeading());
		}
		*/
		// // We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		//return Rotation2d.fromDegrees(360.0 - navx.getYaw());
	}

	public Rotation2d getNavXRotation(){
		return Rotation2d.fromDegrees(navx.getYaw());
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());

		calculatedPose = odometry.update(navx.getRotation2d(), states);
		//SmartDashboard.putNumber("Estimated X", calculatedPose.getX());
		//SmartDashboard.putNumber("Estimated Y", calculatedPose.getY());
		//SmartDashboard.putNumber("\"Estimated\" Rotation", calculatedPose.getRotation().getDegrees());
		estimatedX.setDouble(calculatedPose.getX());
		estimatedY.setDouble(calculatedPose.getY());
		rotation.setDouble(calculatedPose.getRotation().getDegrees());

		gyroPose = new Pose2d(new Translation2d(navx.getDisplacementX(), navx.getDisplacementY()), navx.getRotation2d());
		//SmartDashboard.putNumber("NavX X", gyroPose.getX());
		//SmartDashboard.putNumber("NavX Y", gyroPose.getY());
		NavX.setDouble(gyroPose.getX());
		NavY.setDouble(gyroPose.getY());

		// Not odometry
		SmartDashboard.putNumber("Pigeon Rot", getPigeonRotation().getDegrees());
		/*pigeon.getAccumGyro(xyz);
		SmartDashboard.putNumber("Pigeon X", xyz[0]);
		SmartDashboard.putNumber("Pigeon Y", xyz[1]);
		SmartDashboard.putNumber("Pigeon Z", xyz[2]);*/
		/*odometryTab.add("Pigeon W", xyz[0]);
		odometryTab.add("Pigeon X", xyz[1]);
		odometryTab.add("Pigeon Y", xyz[2]);
		odometryTab.add("Pigeon Z", xyz[3]);*/
	}
}
