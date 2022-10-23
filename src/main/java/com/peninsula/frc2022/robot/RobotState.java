package com.peninsula.frc2022.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2022.config.SwerveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Holds the current physical state of the robot from our sensors. */
@SuppressWarnings ("java:S1104")
public class RobotState {

	public enum GamePeriod {
		AUTO, TELEOP, TESTING, DISABLED
	}

	/* Swerve */
	public Rotation2d gyroHeading = new Rotation2d();
	public double[] moduleEncoderPos = new double[4];
	public double avgModuleVel;
	public SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(
			SwerveConstants.kKinematics,
			new Rotation2d(0));
	public SwerveDriveOdometry driveGhostOdometry = new SwerveDriveOdometry(
			SwerveConstants.kKinematics,
			new Rotation2d(0));
	public Pose2d lastVisionEstimatedPose = new Pose2d();
	public double odometryDeadReckonUpdateTime = 0;
	public double odometryVisionUpdateTime = 0;
	public Field2d m_field = new Field2d();
	public ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
	public ChassisSpeeds chassisRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

	public TimeInterpolatableBuffer<Pose2d> pastPoses = TimeInterpolatableBuffer.createBuffer(1.0);

	/* Joystick */
	public double driverLeftX, driverRightX, driverLeftY, driverRt = 0;
	public boolean operatorAPressed,
			operatorBPressed,
			operatorXPressed,
			operatorYPressed,
			operatorRtPressed,
			operatorLtPressed,
			operatorRbPressed,
			operatorLbPressed,
			operatorDPadLeftPressed,
			operatorDPadRightPressed;
	public double operatorLeftX,
			operatorLeftY,
			operatorRightY,
			operatorRightX;
	public boolean driverAPressed, driverRbPressed;
	public boolean driverLtPressed;

	/* Miscellaneous */
	public GamePeriod gamePeriod = GamePeriod.DISABLED;
	public String gameData;
	public int cycles = 0;
	public double gameTimeS = 0;

	/* Auto */
	public PathPlannerTrajectory currentTrajectory;
	public Pose2d initPose = new Pose2d(0, 0, new Rotation2d(0));

	/* Elevator */
	public double elevatorPosition;
	public double elevatorRadians;

	public double gearDiameter;
	public double gearRotations;
	public double gearToSpoolRatio = 10;
	public double spoolDiameter = gearToSpoolRatio * gearDiameter;
	public double gearToIn = 10 / 3 * gearRotations;
}
