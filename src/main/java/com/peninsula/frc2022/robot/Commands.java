package com.peninsula.frc2022.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.subsystems.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Commands represent what we want the robot to be doing. */
@SuppressWarnings ("java:S1104")
public class Commands {

	/* Routines */
	public List<RoutineBase> routinesWanted = new ArrayList<>();
	public boolean shouldClearCurrentRoutines;
	/* Swerve */
	public Swerve.State swerveWanted;
	public boolean boostWanted;
	public boolean robotCentricWanted;
	public double angleWanted;

	/* Vision */
	public Vision.State visionWanted;

	/* Auto */
	public PathPlannerTrajectory wantedPathPlannerTrajectory;
	public Pose2d driveWantedOdometryPose;
	public Rotation2d driveWantedOdometryPoseRotation;

	public void addWantedRoutines(RoutineBase... wantedRoutines) {
		for (RoutineBase wantedRoutine : wantedRoutines) {
			addWantedRoutine(wantedRoutine);
		}
	}

	public void addWantedRoutine(RoutineBase wantedRoutine) {
		routinesWanted.add(wantedRoutine);
	}

	/* Drive */

	@Override
	public String toString() {
		var log = new StringBuilder();
		log.append("Wanted routines: ");
		for (RoutineBase routine : routinesWanted) {
			log.append(routine).append(" ");
		}
		return log.append("\n").toString();
	}

	public PathPlannerTrajectory getDriveWantedPathPlannerTrajectory() {
		return wantedPathPlannerTrajectory;
	}

	public void setDriveFollowPath(PathPlannerTrajectory pathPlannerTrajectory) {
		wantedPathPlannerTrajectory = pathPlannerTrajectory;
		swerveWanted = Swerve.State.AUTO;
	}

	public void setDriveIdle() {
		swerveWanted = Swerve.State.NEUTRAL;
	}

	public void setClimberPosition(double per) {
	}
}
