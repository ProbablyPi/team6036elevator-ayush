package com.peninsula.frc2022.config;

import com.peninsula.frc2022.util.control.Gains;

public class ElevatorConstants {

	public static double elevatorUpPos = 15000;
	public static double elevatorDownPos = -1000;

	public static double c = 0.08;

	public static Gains elevatorGains = new Gains(0.025, 0, 0, 0, 0, 0);

	/* Physical Constants */

}
