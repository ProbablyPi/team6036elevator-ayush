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


	/** Joysticks */
	public static int kDriverId = 0;
	public static int kOperatorId = 1;

}
