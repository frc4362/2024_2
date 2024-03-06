package com.gemsrobotics.lib;

import com.gemsrobotics.frc2024.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

// this implements Subsystem instead of SubsystemBase so that it may be registered at will or included in a server
public abstract class Limelight implements Subsystem {
	private static final Pose2d NO_POSE = new Pose2d();

	public static class PeriodicIO {
		public Optional<TimestampedVisionUpdate> latestResults = Optional.empty();
	}

	private final StringPublisher m_statePublisher;
	private final String m_name;
	private final PeriodicIO m_periodicIO;

	protected Limelight(final String name) {
		m_name = name;
		m_periodicIO = new PeriodicIO();

		String nt_key = "limelight_subsystem";

		if (!name.isEmpty()) {
			nt_key += "_" + name;
		}

		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(nt_key);
		m_statePublisher = myTable.getStringTopic("state").publish();

	}

	protected Limelight() {
		this("");
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("valid", LimelightHelpers.getTV(m_name));

		if (!LimelightHelpers.getTV(m_name)) {
			m_periodicIO.latestResults = Optional.empty();
			m_statePublisher.set("no update (invalid)");
			return;
		}

		final var newResults = Optional.ofNullable(LimelightHelpers.getLatestResults(m_name).targetingResults);

		if (newResults.isEmpty() || !newResults.get().valid) {
			m_periodicIO.latestResults = Optional.empty();
			m_statePublisher.set("no update (deserialization invalid)");
			return;
		}

		if (newResults.map(results -> results.targets_Fiducials == null || results.targets_Fiducials.length < 2).orElse(true)) {
			m_periodicIO.latestResults = Optional.empty();
			m_statePublisher.set("no update (not enough targets");
			return;
		}

		final LimelightHelpers.Results results = newResults.get();
		if (NO_POSE.getTranslation().getDistance(results.getBotPose2d_wpiBlue().getTranslation()) == 0.0) {
			m_statePublisher.set("pose is empty");
			m_periodicIO.latestResults = Optional.empty();
			return;
		}

		final double timestampSeconds = Timer.getFPGATimestamp() - getTotalLatencySecondsInternal(results);
		final Pose2d robotPose = results.getBotPose2d_wpiBlue();
		final List<Pose2d> camToTagPoses = Arrays.stream(results.targets_Fiducials).map(LimelightHelpers.LimelightTarget_Fiducial::getTargetPose_CameraSpace2D).toList();
		final List<Double> targetDistances = Arrays.stream(results.targets_Fiducials).map(target ->
				target.getTargetPose_RobotSpace2D().getTranslation().getNorm()).toList();

		m_periodicIO.latestResults = Optional.of(new TimestampedVisionUpdate(timestampSeconds, robotPose, camToTagPoses, targetDistances));
	}

	private static double getTotalLatencySecondsInternal(final LimelightHelpers.Results results) {
		return (results.latency_capture + results.latency_pipeline + results.latency_jsonParse) / 1_000.0;
	}

	private static int getTargetCountInternal(final LimelightHelpers.Results results) {
		return results.targets_Fiducials.length;
	}

	public Optional<TimestampedVisionUpdate> getUpdate() {
		return m_periodicIO.latestResults;
	}
}
