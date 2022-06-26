package com.peninsula.frc2022.robot;

import java.util.*;

import com.peninsula.frc2022.auto.*;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.RoutineManager;
import com.peninsula.frc2022.subsystems.*;
import com.peninsula.frc2022.subsystems.SubsystemBase;
import com.peninsula.frc2022.util.LoopOverrunDebugger;
import com.peninsula.frc2022.util.Util;
import com.peninsula.frc2022.util.csvlogger.CSVWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

@SuppressWarnings ("java:S1104")
public class Robot extends TimedRobot {

	public static final double kPeriod = 0.02;
	private static final String kLoggerTag = Util.classToJsonName(Robot.class);

	RoutineBase autoChosen;

	private static final boolean kCanUseHardware = RobotBase.isReal() || !System.getProperty("os.name").startsWith("Mac");

	private final RobotState mRobotState = new RobotState();
	private final Control mOperatorInterface = new Control();
	private final RoutineManager mRoutineManager = new RoutineManager();
	private final HardwareReader mHardwareReader = new HardwareReader();
	private final HardwareWriter mHardwareWriter = new HardwareWriter();
	private final Commands mCommands = new Commands();

	/* Subsystems */
	private final Swerve mSwerve = Swerve.getInstance();
	private final Vision mVision = Vision.getInstance();

	private Set<SubsystemBase> mSubsystems = Set.of(mSwerve, mVision);

	public static final LoopOverrunDebugger sLoopDebugger = new LoopOverrunDebugger("teleop", kPeriod);

	public Robot() {
		super(kPeriod);
	}

	@Override
	public void robotInit() {
		mHardwareWriter.configureHardware(mSubsystems);
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void disabledInit() {
		mRobotState.gamePeriod = RobotState.GamePeriod.DISABLED;
		resetCommandsAndRoutines();
		CSVWriter.write();
	}

	@Override
	public void autonomousInit() {
		startStage(RobotState.GamePeriod.AUTO);
		mCommands.addWantedRoutine(autoChosen);
		HardwareAdapter.SwerveHardware.getInstance().gyro.zeroYaw();
	}

	private void startStage(RobotState.GamePeriod period) {
		mRobotState.gamePeriod = period;
		resetCommandsAndRoutines();
		CSVWriter.cleanFile();
		CSVWriter.resetTimer();
	}

	@Override
	public void teleopInit() {
		startStage(RobotState.GamePeriod.TELEOP);

	}

	@Override
	public void testInit() {
		startStage(RobotState.GamePeriod.TESTING);
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void simulationPeriodic() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousPeriodic() {
		sLoopDebugger.reset();
		readRobotState();
		mRoutineManager.update(mCommands, mRobotState);
		updateSubsystemsAndApplyOutputs();
		sLoopDebugger.finish();
	}

	@Override
	public void teleopPeriodic() {
		sLoopDebugger.reset();
		readRobotState();
		mOperatorInterface.updateCommands(mCommands, mRobotState);
		mRoutineManager.update(mCommands, mRobotState);
		updateSubsystemsAndApplyOutputs();
		sLoopDebugger.finish();
	}

	@Override
	public void testPeriodic() {
		teleopPeriodic();
	}

	private void resetCommandsAndRoutines() {
		mOperatorInterface.reset(mCommands);
		mRoutineManager.clearRunningRoutines();
		updateSubsystemsAndApplyOutputs();
	}

	private void readRobotState() {
		mHardwareReader.readState(mSubsystems, mRobotState);
	}

	private void resetOdometryIfWanted() {
		Pose2d wantedPose = mCommands.driveWantedOdometryPose;
		Rotation2d wantedPoseRotation = mCommands.driveWantedOdometryPoseRotation;
		if (wantedPose != null && wantedPoseRotation != null) {
			mRobotState.driveOdometry.resetPosition(wantedPose, wantedPoseRotation);
			mRobotState.driveGhostOdometry.resetPosition(wantedPose, wantedPoseRotation);
			mCommands.driveWantedOdometryPose = null;
			mCommands.driveWantedOdometryPoseRotation = null;
		}
	}

	private void updateSubsystemsAndApplyOutputs() {
		resetOdometryIfWanted();
		for (SubsystemBase subsystem : mSubsystems) {
			subsystem.update(mCommands, mRobotState);
			sLoopDebugger.addPoint(subsystem.getName());
		}
		mHardwareWriter.writeHardware(mSubsystems, mRobotState);
		sLoopDebugger.addPoint("updateSubsystemsAndApplyOutputs");
	}
}
