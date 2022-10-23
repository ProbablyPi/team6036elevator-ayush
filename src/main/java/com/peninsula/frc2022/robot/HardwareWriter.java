package com.peninsula.frc2022.robot;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.*;
import com.peninsula.frc2022.config.ElevatorConstants;
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
		if (enabledSubsystems.contains(Elevator.getInstance())) configureElevatorHardware();
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
		updateElevator();
	}

	private void configureElevatorHardware() {
		var hardware = HardwareAdapter.ElevatorHardware.getInstance();

		hardware.elevatorMotorOne.follow(hardware.elevatorMotorOne);
		hardware.elevatorMotorOne.setInverted(false);
		hardware.elevatorMotorTwo.setInverted(true);

		hardware.elevatorMotorOne.config_kP(0, ElevatorConstants.elevatorGains.p);
		hardware.elevatorMotorOne.config_kI(0, ElevatorConstants.elevatorGains.i);
		hardware.elevatorMotorOne.config_kD(0, ElevatorConstants.elevatorGains.d);
		hardware.elevatorMotorOne.config_kF(0, ElevatorConstants.elevatorGains.f);

		hardware.elevatorMotorTwo.config_kP(0, ElevatorConstants.elevatorGains.p);
		hardware.elevatorMotorTwo.config_kI(0, ElevatorConstants.elevatorGains.i);
		hardware.elevatorMotorTwo.config_kD(0, ElevatorConstants.elevatorGains.d);
		hardware.elevatorMotorTwo.config_kF(0, ElevatorConstants.elevatorGains.f);

		hardware.elevatorMotorOne.setNeutralMode(NeutralMode.Brake);
		hardware.elevatorMotorTwo.setNeutralMode(NeutralMode.Brake);

		hardware.elevatorMotorOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 1000);
		hardware.elevatorMotorTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 1000);
		hardware.elevatorMotorOne.setSelectedSensorPosition(0.0);
		hardware.elevatorMotorTwo.setSelectedSensorPosition(0.0);
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

	private void updateElevator() {
		var hardware = HardwareAdapter.ElevatorHardware.getInstance();
		var outputs = Elevator.getInstance().getOutputs1();

		hardware.elevatorMotorOne.setOutput(outputs);
	}
}
