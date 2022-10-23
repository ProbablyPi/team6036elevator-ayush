package com.peninsula.frc2022.robot;

import java.util.Set;

import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.robot.HardwareAdapter.JoystickHardware;
import com.peninsula.frc2022.subsystems.*;
import com.peninsula.frc2022.util.Util;
import com.peninsula.frc2022.util.control.Spark;
import com.peninsula.frc2022.util.control.Talon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HardwareReader {

	private static final String kLoggerTag = Util.classToJsonName(HardwareReader.class);
	//	private static final int kYawIndex = 0, kYawAngularVelocityIndex = 2;

	public HardwareReader() {
	}

	/**
	 * Takes all of the sensor data from the hardware, and unwraps it into the current
	 * {@link RobotState}.
	 */
	void readState(Set<SubsystemBase> enabledSubsystems, RobotState state) {
		readGameAndFieldState(state);
		if (state.gamePeriod == RobotState.GamePeriod.TELEOP) readJoystickState(state);
		if (enabledSubsystems.contains(Swerve.getInstance())) readSwerveState(state);
		if (enabledSubsystems.contains(Vision.getInstance())) readVisionState(state);
	}

	private void readGameAndFieldState(RobotState state) {
		state.gameData = DriverStation.getGameSpecificMessage();
		state.cycles += 1;
		state.gameTimeS = Timer.getFPGATimestamp();
	}

	private void readElevatorState(RobotState state) {
		var hardware = HardwareAdapter.ElevatorHardware.getInstance();
		state.elevatorPosition = hardware.elevatorMotorOne.getSelectedSensorPosition();
		state.elevatorRadians = state.elevatorPosition / (30 * 2048) * 6.2831853072;
		SmartDashboard.putNumber("Intake Arm", state.elevatorPosition);
	}

	private void readSwerveState(RobotState state) {
		var hardware = HardwareAdapter.SwerveHardware.getInstance();
		state.gyroHeading = (hardware.gyro.isMagnetometerCalibrated()) ? Rotation2d.fromDegrees(hardware.gyro.getFusedHeading()) : Rotation2d.fromDegrees(360.0 - hardware.gyro.getYaw());
		for (int i = 0; i < state.moduleEncoderPos.length; i++) {
			state.moduleEncoderPos[i] = hardware.modules[i].getSteerAngle();
		}

		var outputs = Swerve.getInstance().getOutputs().getStates();
		var outputsR = Swerve.getInstance().getOutputs().getRotation2d(); //TODO: only for simulation

		SwerveModuleState[] moduleStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) moduleStates[i] = new SwerveModuleState();

		/* Read real module encoder speeds */
		moduleStates[0].angle = outputs[0].angle;
		moduleStates[0].speedMetersPerSecond = Math.abs(
				hardware.modules[0].getDriveVelocity());

		moduleStates[1].angle = outputs[1].angle;
		moduleStates[1].speedMetersPerSecond = Math.abs(
				hardware.modules[1].getDriveVelocity());

		moduleStates[2].angle = outputs[2].angle;
		moduleStates[2].speedMetersPerSecond = Math.abs(
				hardware.modules[2].getDriveVelocity());

		moduleStates[3].angle = outputs[3].angle;
		moduleStates[3].speedMetersPerSecond = Math.abs(
				hardware.modules[3].getDriveVelocity());

		//+x is shooter side, +y is side with PDP
		state.chassisRelativeSpeeds = SwerveConstants.kKinematics.toChassisSpeeds(outputs);
//		state.chassisRelativeSpeeds = SwerveConstants.kKinematics.toChassisSpeeds(moduleStates);

		//+x is towards alliance station, +y is right of driver station
		state.fieldRelativeSpeeds = new ChassisSpeeds(
				state.gyroHeading.getCos() * state.chassisRelativeSpeeds.vxMetersPerSecond -
						state.gyroHeading.getSin() * state.chassisRelativeSpeeds.vyMetersPerSecond,
				state.gyroHeading.getSin() * state.chassisRelativeSpeeds.vxMetersPerSecond +
						state.gyroHeading.getCos() * state.chassisRelativeSpeeds.vyMetersPerSecond,
				state.chassisRelativeSpeeds.omegaRadiansPerSecond);

		state.gyroHeading = state.gyroHeading.plus(state.initPose.getRotation());
		state.driveOdometry.update(outputsR, outputs); // Simulation
//		state.driveOdometry.update(state.gyroHeading, moduleStates); // On ground

		state.odometryDeadReckonUpdateTime = state.gameTimeS;
		state.pastPoses.addSample(state.odometryDeadReckonUpdateTime, state.driveOdometry.getPoseMeters());

		if (state.currentTrajectory != null) {

			/* Field 2D update */
			state.m_field.getObject("traj").setTrajectory(state.currentTrajectory);
		}

		SmartDashboard.putData(state.m_field);

		state.m_field.setRobotPose(state.driveOdometry.getPoseMeters());

	}

	private void readJoystickState(RobotState state) {
		var hardware = JoystickHardware.getInstance();

		state.driverLeftX = Util.handleDeadBand(hardware.driverXboxController.getLeftX(), 0.09);
		state.driverLeftY = Util.handleDeadBand(hardware.driverXboxController.getLeftY(), 0.09);
		state.driverRightX = Util.handleDeadBand(hardware.driverXboxController.getRightX(), 0.04);
		state.driverRt = hardware.driverXboxController.getRightTriggerAxis();
		state.operatorAPressed = hardware.operatorXboxController.getAButton();
		state.operatorRtPressed = hardware.operatorXboxController.getRightTriggerAxis() > 0.5;
		state.operatorLtPressed = hardware.operatorXboxController.getLeftTriggerAxis() > 0.5;
		state.operatorRbPressed = hardware.operatorXboxController.getRightBumper();
		state.operatorLbPressed = hardware.operatorXboxController.getLeftBumper();
		state.operatorDPadLeftPressed = hardware.operatorXboxController.getLeftStickButton();
		state.operatorDPadRightPressed = hardware.operatorXboxController.getRightStickButton();
		state.driverAPressed = hardware.driverXboxController.getAButton();
		state.driverRbPressed = hardware.driverXboxController.getRightBumper();
		state.operatorLeftX = Util.handleDeadBand(hardware.operatorXboxController.getLeftX(), 0.09);
		state.operatorLeftY = Util.handleDeadBand(hardware.operatorXboxController.getLeftY(), 0.09);
		state.operatorRightX = Util.handleDeadBand(hardware.operatorXboxController.getRightX(), 0.2);
		state.operatorRightY = Util.handleDeadBand(hardware.operatorXboxController.getRightY(), 0.2);
		state.operatorAPressed = hardware.operatorXboxController.getAButton();
		state.operatorBPressed = hardware.operatorXboxController.getBButton();
		state.operatorXPressed = hardware.operatorXboxController.getXButton();
		state.operatorYPressed = hardware.operatorXboxController.getYButton();
		state.driverLtPressed = hardware.driverXboxController.getLeftTriggerAxis() > 0.5;
	}

	private void readVisionState(RobotState state) {
		var hardware = HardwareAdapter.VisionHardware.getInstance();

	}

	private void checkSparkFaults(Spark spark) {
		//    if (mRobotConfig.checkFaults) {
		//      boolean wasAnyFault = false;
		//      for (var value : FaultID.values()) {
		//        boolean isFaulted = spark.getStickyFault(value);
		//        if (isFaulted) {
		//          Log.error(kLoggerTag, String.format("Spark %d fault: %s", spark.getDeviceId(),
		// value));
		//          wasAnyFault = true;
		//        }
		//      }
		//      if (wasAnyFault) {
		//        spark.clearFaults();
		//      }
		//    }
	}

	private void checkTalonFaults(Talon talon) {
		//    if (mRobotConfig.checkFaults) {
		//      var faults = new StickyFaults();
		//      talon.getStickyFaults(faults);
		//      if (faults.hasAnyFault()) {
		//        Log.error(kLoggerTag, String.format("%s faults: %s", talon.getName(), faults));
		//        talon.clearStickyFaults();
		//      }
		//    }
	}

	//  private void checkFalconFaults(Falcon falcon) {
	//    if (mRobotConfig.checkFaults) {
	//      var faults = new StickyFaults();
	//      falcon.getStickyFaults(faults);
	//      if (faults.hasAnyFault()) {
	//        Log.error(kLoggerTag, String.format("%s faults: %s", falcon.getName(), faults));
	//        falcon.clearStickyFaults();
	//      }
	//    }
	//  }

}
