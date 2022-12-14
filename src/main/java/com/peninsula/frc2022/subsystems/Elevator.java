package com.peninsula.frc2022.subsystems;

import com.peninsula.frc2022.config.ElevatorConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.util.control.ControllerOutput;

public class Elevator extends SubsystemBase {

	public enum State {
		UP, DOWN
	}

	private ControllerOutput mOutputs1 = new ControllerOutput();
	private static Elevator sInstance = new Elevator();

	public double tickPos = 0;

	public boolean wantedReset = false;

	private Elevator() {
	}

	public static Elevator getInstance() {
		return sInstance;
	}

	public ControllerOutput getOutputs1() {
		return mOutputs1;
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		if (commands.wantedReset) {
			this.wantedReset = commands.wantedReset;
			commands.wantedReset = false;
		}
		switch (commands.elevatorWanted) {
			case UP:
				System.out.println("UP");
				mOutputs1.setTargetPosition(ElevatorConstants.elevatorUpPos, ElevatorConstants.elevatorGains);
				tickPos = ElevatorConstants.elevatorDownPos;

				break;
			case DOWN:
				System.out.println("DOWN");
				if (commands.wantedReset) {
					break;
				}
				mOutputs1.setTargetPosition(ElevatorConstants.elevatorUpPos, ElevatorConstants.elevatorGains);
				tickPos = ElevatorConstants.elevatorDownPos;
				break;
		}

	}
}
