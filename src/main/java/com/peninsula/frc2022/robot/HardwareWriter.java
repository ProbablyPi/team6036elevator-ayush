package com.peninsula.frc2022.robot;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.*;
import com.peninsula.frc2022.subsystems.*;

import org.photonvision.common.hardware.VisionLEDMode;

public class HardwareWriter {

	public static final int
	// Blocks config calls for specified timeout
	kTimeoutMs = 150,
			// Different from slot index.
			// 0 for Primary closed-loop. 1 for auxiliary closed-loop.
			kPidIndex = 0;
//	private static final String kLoggerTag = Util.classToJsonName(HardwareWriter.class);
//	public static final double kVoltageCompensation = 12.0;
//	public static final SupplyCurrentLimitConfiguration k30AmpCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 30.0, 35.0, 1.0);

	void configureHardware(Set<SubsystemBase> enabledSubsystems) {
		if (enabledSubsystems.contains(Swerve.getInstance())) configureSwerveHardware();
		if (enabledSubsystems.contains(Vision.getInstance())) configureVisionHardware();
	}

	private void configureSwerveHardware() {
	}

	private void configureVisionHardware() {
		var hardware = HardwareAdapter.VisionHardware.getInstance();
		hardware.camera.setPipelineIndex(0);
		hardware.camera.setLED(VisionLEDMode.kOn);
	}

	/** Updates the hardware to run with output values of {@link SubsystemBase}'s. */
	void writeHardware(Set<SubsystemBase> enabledSubsystems, @ReadOnly RobotState robotState) {
		if (enabledSubsystems.contains(Swerve.getInstance())) updateSwerve();
		updateJoysticks();
	}

	private void updateSwerve() {
		var hardware = HardwareAdapter.SwerveHardware.getInstance();
		var outputs = Swerve.getInstance().getOutputs();
		if (Swerve.getInstance().getZero()) hardware.gyro.zeroYaw();
		if (!outputs.isIdle()) {
			hardware.FL.set(outputs.voltages()[0], outputs.steerAngles()[0]);
			hardware.FR.set(outputs.voltages()[1], outputs.steerAngles()[1]);
			hardware.BL.set(outputs.voltages()[2], outputs.steerAngles()[2]);
			hardware.BR.set(outputs.voltages()[3], outputs.steerAngles()[3]);
		}
	}

	private void updateJoysticks() {
		var hardware = HardwareAdapter.JoystickHardware.getInstance();
	}
}
