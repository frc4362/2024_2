package com.gemsrobotics.frc2024;

import com.gemsrobotics.lib.TimestampedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import java.util.Optional;

public class NewTargetServer implements Subsystem {
	private static NewTargetServer INSTANCE;

	public static NewTargetServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new NewTargetServer();
		}

		return INSTANCE;
	}

	public static class PeriodicIO {
		public Optional<LimelightHelpers.PoseEstimate> estimate = Optional.empty();
	}

	private final StringPublisher m_statePublisher;
	private final PeriodicIO m_periodicIO;
	private final String m_name;

	private NewTargetServer() {
		m_name = "";

		String nt_key = "limelight_subsystem";
		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(nt_key);
		m_statePublisher = myTable.getStringTopic("state").publish();

		m_periodicIO = new PeriodicIO();
	}

	@Override
	public void periodic() {
		if (!LimelightHelpers.getTV(m_name)) {
			m_statePublisher.set("no target");
			m_periodicIO.estimate = Optional.empty();
			return;
		}

		// TODO figure this out later, frame of reference may not be the same as choreo
		final var newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);

		if (newEstimate.tagCount < 2) {
			m_statePublisher.set("not enough tags");
			m_periodicIO.estimate = Optional.empty();
			return;
		}

		if (newEstimate.pose.getTranslation().getNorm() == 0.0) {
			m_statePublisher.set("not a real pose estimate");
			m_periodicIO.estimate = Optional.empty();
			return;
		}

		m_statePublisher.set("valid vision reading");
		m_periodicIO.estimate = Optional.of(newEstimate);
	}

	public Optional<LimelightHelpers.PoseEstimate> getLatestEstimate() {
		return m_periodicIO.estimate;
	}

	private static double measuredMetersToRealMeters(final double measured) {
		return 0.887 * measured + 0.038;
	}

	private static final Rotation2d LIMELIGHT_PITCH = Rotation2d.fromDegrees(10.0);
	private static final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(14.875);
	private static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(60.75);

	public Optional<Transform2d> getFallbackCameraToTarget() {
		if (!LimelightHelpers.getTV(m_name)) {
			return Optional.empty();
		}

		// find angle of targets in frame
		final var targetOffsetAngle = LIMELIGHT_PITCH.plus(Rotation2d.fromDegrees(LimelightHelpers.getTY(m_name)));
		final var distanceMeters = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT_METERS) / Math.tan(targetOffsetAngle.getRadians());
		final var realDistanceMeters = measuredMetersToRealMeters(distanceMeters);
		final var cameraToTarget = new Transform2d(realDistanceMeters, 0.0, Rotation2d.fromDegrees(-1.0 * LimelightHelpers.getTX(m_name)));
		return Optional.of(cameraToTarget);
	}
}
