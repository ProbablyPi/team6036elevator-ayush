package com.peninsula.frc2022.config;

import com.peninsula.frc2022.util.control.Gains;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

	public static final double kDriveRatio = 6.86;
	public static final double kWheelRadius = Units.inchesToMeters(2);
	public static final double kOffsetX = Units.inchesToMeters(28.0 / 2);
	public static final double kOffsetY = Units.inchesToMeters(28.0 / 2);

	public static double FRo = (4.950 - 3.128 + 2 * Math.PI) % (2 * Math.PI);
	public static double FLo = (3.775 - 3.155 + 2 * Math.PI) % (2 * Math.PI);
	public static double BRo = (2.046 - 0.034 + 2 * Math.PI) % (2 * Math.PI);
	public static double BLo = (2.295 - 6.267 + 2 * Math.PI + 2 * Math.PI) % (2 * Math.PI);

//	public static Gains kCloseAlignGains = new Gains(-0.095, -0.1, -0.01, 0, Math.PI / 10, 0.0);
	public static Gains kCloseAlignGains = new Gains(0.065, 0.00, 0.0, 0, 0, 0.0);
//	public static Gains kCloseAlignGains = new Gains(0.075, 0.00, 0.0, 0, 0, 0.0);
//	public static Gains kFarAlignGains = new Gains(0.105, 0.0, 0.0, 0, 0, 0.0)

	public static Gains kSetAngleGains = new Gains(1, 0, 0, 0, 0, 0);

	static Translation2d kFLPos = new Translation2d(kOffsetX, kOffsetY);
	static Translation2d kFRPos = new Translation2d(kOffsetX, -kOffsetY);
	static Translation2d kBLPos = new Translation2d(-kOffsetX, kOffsetY);
	static Translation2d kBRPos = new Translation2d(-kOffsetX, -kOffsetY);

	public static final double kMaxVoltage = 12.0;

	public static final double kTeleopMaxTransVel = 3.5;
	public static final double kTeleopBoostMaxTransVel = 5.0;
	public static final double kTeleopMaxRotVel = 4;
	public static final double kTeleopBoostMaxRotVel = 5;
	public static final double kMaxVelocityMetersPerSecond = 6380.0 / 60.0 * SdsModuleConfigurations.MK3_FAST_WITH_TREAD.getDriveReduction() * SdsModuleConfigurations.MK3_FAST_WITH_TREAD.getWheelDiameter() * Math.PI;

	public static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kFLPos, kFRPos, kBLPos, kBRPos);
}
