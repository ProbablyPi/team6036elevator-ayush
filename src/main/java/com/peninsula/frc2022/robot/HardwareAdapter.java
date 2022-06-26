package com.peninsula.frc2022.robot;

import com.kauailabs.navx.frc.AHRS;
import com.peninsula.frc2022.config.PortConstants;
import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.util.config.Configs;
import com.peninsula.frc2022.util.control.Falcon;
import com.peninsula.frc2022.util.control.Spark;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.*;

import org.photonvision.PhotonCamera;

/**
 * Represents all hardware components of the robot. Singleton class. Should only be used in robot
 * package. Subdivides hardware into subsystems.
 */
public class HardwareAdapter {

	/**
	 * 4 Falcon 500s (controlled by Talon FX), 1 Pigeon IMU Gyro connected via Talon SRX data cable.
	 */
	public static class SwerveHardware {

		private static SwerveHardware sInstance;

		// CW Starting from top left module
		// <Drive Motor, Turn Motor, Encoder>
		public final SwerveModule FL, FR, BL, BR;
		public final SwerveModule[] modules;
		public final AHRS gyro;

		private SwerveHardware() {

			FL = Mk3SwerveModuleHelper.createFalcon500(
					Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
					PortConstants.driveFL.kD,
					PortConstants.driveFL.kT,
					PortConstants.driveFL.kE,
					-SwerveConstants.FLo);
			FR = Mk3SwerveModuleHelper.createFalcon500(
					Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
					PortConstants.driveFR.kD,
					PortConstants.driveFR.kT,
					PortConstants.driveFR.kE,
					-SwerveConstants.FRo);
			BL = Mk3SwerveModuleHelper.createFalcon500(
					Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
					PortConstants.driveBL.kD,
					PortConstants.driveBL.kT,
					PortConstants.driveBL.kE,
					-SwerveConstants.BLo);
			BR = Mk3SwerveModuleHelper.createFalcon500(
					Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
					PortConstants.driveBR.kD,
					PortConstants.driveBR.kT,
					PortConstants.driveBR.kE,
					-SwerveConstants.BRo);
			modules = new SwerveModule[] { FL, FR, BL, BR };
			gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
		}

		static SwerveHardware getInstance() {
			if (sInstance == null) sInstance = new SwerveHardware();
			return sInstance;
		}
	}

	static class LightingHardware {

		private static LightingHardware sInstance;

		final AddressableLED ledStrip;

		private LightingHardware() {
			ledStrip = new AddressableLED(PortConstants.kLighting);
		}

		static LightingHardware getInstance() {
			if (sInstance == null) sInstance = new LightingHardware();
			return sInstance;
		}
	}

	/** 1 NEO */
	static class IntakeHardware {

		private static IntakeHardware sInstance;

		final Falcon motor;

		private IntakeHardware() {
			motor = new Falcon(PortConstants.kIntakeId, "intake");
		}

		static IntakeHardware getInstance() {
			if (sInstance == null) sInstance = new IntakeHardware();
			return sInstance;
		}
	}

	/** 1 NEO for intake indexer motor 1 Falcon for kicker wheel motor */
	static class IndexerHardware {

		private static IndexerHardware sInstance;

		final Spark intakeIndexerMotor;
		final Falcon kickerMotor;
		final DigitalInput intakeIndexerSensor;
		final DigitalInput stopperSensor;
		final DigitalInput firstPosSensor;

		private IndexerHardware() {
			intakeIndexerMotor = new Spark(PortConstants.kIntakeIndexerId, "intakeIndexer");
			kickerMotor = new Falcon(PortConstants.kKickerId, "kicker");
			intakeIndexerSensor = new DigitalInput(PortConstants.kIntakeIndexerSensorId);
			stopperSensor = new DigitalInput(PortConstants.kStopperSensorId);
			firstPosSensor = new DigitalInput(PortConstants.kFirstPosSensorId);
		}

		static IndexerHardware getInstance() {
			if (sInstance == null) sInstance = new IndexerHardware();
			return sInstance;
		}
	}

	/** 2 Falcon Shooter */
	static class ShooterHardware {

		private static ShooterHardware sInstance;
		Falcon shooterMasterMotor;
		Falcon shooterSlaveMotor;
		Falcon hoodMotor;

		private ShooterHardware() {
			shooterMasterMotor = new Falcon(PortConstants.kShooterMaster, "shooterMaster");
			shooterSlaveMotor = new Falcon(PortConstants.kShooterSlave, "shooterSlave");
			hoodMotor = new Falcon(PortConstants.kShooterHood, "hoodSlave");
		}

		static ShooterHardware getInstance() {
			if (sInstance == null) sInstance = new ShooterHardware();
			return sInstance;
		}
	}

	static class ClimberHardware {

		private static ClimberHardware sInstance;
		final Spark tilterMotorRight, tilterMotorLeft;
		final Falcon pullerMotorRight, pullerMotorLeft;
		final DigitalInput pullerStopRight, pullerStopLeft;
//		final RelativeEncoder tilterMotorEncoder;

		private ClimberHardware() {
			tilterMotorRight = new Spark(PortConstants.kClimberRightTilter, "Right Climber Tilter");
//			tilterEncoderRight = new DutyCycleEncoder(PortConstants.kClimberRightTilterEncoder);

			tilterMotorLeft = new Spark(PortConstants.kClimberLeftTilter, "Left Climber Tilter");
//			tilterEncoderLeft = new DutyCycleEncoder(PortConstants.kClimberLeftTilterEncoder);

			pullerMotorRight = new Falcon(PortConstants.kClimberRightPuller, "Right Climber Puller");
//			pullerEncoderRight = new DutyCycleEncoder(PortConstants.kClimberRightPullerEncoder);

			pullerMotorLeft = new Falcon(PortConstants.kClimberLeftPuller, "Left Climber Puller");
//			pullerEncoderLeft = new DutyCycleEncoder(PortConstants.kClimberLeftPullerEncoder);

			pullerStopRight = new DigitalInput(PortConstants.kClimberRightPullerStop);
			pullerStopLeft = new DigitalInput(PortConstants.kClimberLeftPullerStop);

//			tilterMotorEncoder = tilterMotorLeft.getAlternateEncoder(8192);
		}

		static ClimberHardware getInstance() {
			if (sInstance == null) sInstance = new ClimberHardware();
			return sInstance;
		}
	}

	static class VisionHardware {

		private static VisionHardware sInstance;

		PhotonCamera camera;

		private VisionHardware() {
			camera = new PhotonCamera("gloworm");
		}

		static VisionHardware getInstance() {
			if (sInstance == null) sInstance = new VisionHardware();
			return sInstance;
		}
	}

	/** 2 Joysticks, 1 Xbox Controller */
	static class JoystickHardware {

		private static JoystickHardware sInstance;

		final XboxController driverXboxController;
		final XboxController operatorXboxController;

		private JoystickHardware() {
			driverXboxController = new XboxController(PortConstants.kDriverId);
			operatorXboxController = new XboxController(PortConstants.kOperatorId);
		}

		static JoystickHardware getInstance() {
			if (sInstance == null) sInstance = new JoystickHardware();
			return sInstance;
		}
	}

	private static final PortConstants sPortConstants = Configs.get(PortConstants.class);

	private HardwareAdapter() {
	}
}
