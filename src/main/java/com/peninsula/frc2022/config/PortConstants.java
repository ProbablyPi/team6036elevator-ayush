package com.peninsula.frc2022.config;

import com.peninsula.frc2022.util.config.ConfigBase;

@SuppressWarnings ("java:S1104")
public class PortConstants extends ConfigBase {

	/** Drivetrain */
	public static class driveFR {

		public static final int kD = 2;
		public static final int kT = 1;
		public static final int kE = 9;
	}

	public static final class driveFL {

		public static final int kD = 4;
		public static final int kT = 3;
		public static final int kE = 10;
	}

	public static final class driveBL {

		public static final int kD = 6;
		public static final int kT = 5;
		public static final int kE = 11;
	}

	public static final class driveBR {

		public static final int kD = 8;
		public static final int kT = 7;
		public static final int kE = 12;
	}

	/** Intake */
	public static int kIntakeId = 31;

	/** Indexer */
	public static int kIntakeIndexerId = 36;

	public static int kKickerId = 12;
	public static int kStopperSensorId = 2;
	public static int kIntakeIndexerSensorId = 0;
	public static int kFirstPosSensorId = 1;

	/** Shooter */
	public static int kShooterMaster = 21;

	public static int kShooterSlave = 22;
	public static int kShooterHood = 23;

	/** Climber * */
	public static int kClimberRightTilter = 51;
	public static int kClimberRightPuller = 55;
//	public static int kClimberRightTilterEncoder = 6;
//	public static int kClimberRightPullerEncoder = 4;

	public static int kClimberLeftTilter = 53;
	public static int kClimberLeftPuller = 58;
//	public static int kClimberLeftTilterEncoder = 7;
//	public static int kClimberLeftPullerEncoder = 3;

	public static int kClimberLeftPullerStop = 4;
	public static int kClimberRightPullerStop = 7;

	/** Joysticks */
	public static int kDriverId = 0;
	public static int kOperatorId = 1;

	/** LED Strip */
	public static int kLighting = 0;
}
