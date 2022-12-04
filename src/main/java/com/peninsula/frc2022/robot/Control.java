package com.peninsula.frc2022.robot;

import com.peninsula.frc2022.subsystems.*;

import static com.peninsula.frc2022.subsystems.Elevator.State.DOWN;
import static com.peninsula.frc2022.subsystems.Elevator.State.UP;

/** Used to produce {@link Commands}'s from human input. Should only be used in robot package. */
public class Control {

	public static final double kDeadBand = 0.05;
	public static final int kOnesTimesZoomAlignButton = 3, kTwoTimesZoomAlignButton = 4;

	/** Modifies commands based on operator input devices. */
	void updateCommands(Commands commands, @ReadOnly RobotState state) {
		updateDriveCommands(commands, state);
		updateSuperstructureCommands(commands, state);
		updateElevator(commands, state);
	}

	private void updateDriveCommands(Commands commands, RobotState state) {
		commands.swerveWanted = Swerve.State.TELEOP;
		commands.boostWanted = (state.driverRt > 0.5);
		commands.robotCentricWanted = state.driverLtPressed;
	}

	private void updateElevator(Commands commands, RobotState state) {
		if (state.driverAPressed) {
			commands.elevatorWanted(UP);
		} else if (state.driverRbPressed) {
			commands.elevatorWanted(DOWN);
		}
		commands.wantedReset = state.limitSwtitchHit;
	}

	private void updateSuperstructureCommands(Commands commands, RobotState state) {

	}

	public void reset(Commands commands) {
		commands.routinesWanted.clear();
		commands.swerveWanted = Swerve.State.TELEOP;
		commands.boostWanted = false;
		commands.visionWanted = Vision.State.ON;
	}
}
