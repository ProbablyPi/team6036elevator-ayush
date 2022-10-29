package com.peninsula.frc2022.robot;

import com.kauailabs.navx.frc.AHRS;
import com.peninsula.frc2022.config.PortConstants;
import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.util.config.Configs;
import com.peninsula.frc2022.util.control.Falcon;
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

	static class ElevatorHardware {

		private static ElevatorHardware sInstance;

		final Falcon elevatorMotorOne;
		final Falcon elevatorMotorTwo;

		final DigitalInput limitSwitch;

		private ElevatorHardware() {
			elevatorMotorOne = new Falcon(PortConstants.kElevatorMotorOneID, "elevatorOne");
			elevatorMotorTwo = new Falcon(PortConstants.kElevatorMotorTwoID, "elevatorTwo");
			limitSwitch = new DigitalInput(PortConstants.kLimitSwitchID);
		}

		static ElevatorHardware getInstance() {
			if (sInstance == null) sInstance = new ElevatorHardware();
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
