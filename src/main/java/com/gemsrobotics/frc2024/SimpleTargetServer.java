package com.gemsrobotics.frc2024;

import com.gemsrobotics.lib.Limelight;
import com.gemsrobotics.lib.TimestampedValue;
import com.gemsrobotics.lib.TimestampedVisionUpdate;
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

public class SimpleTargetServer implements Subsystem {
	private static final Rotation2d LIMELIGHT_PITCH = Rotation2d.fromDegrees(10.0);
	private static final double LIMELIGHT_HEIGHT_METERS = Units.inchesToMeters(14.875);
	private static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(60.75);

	private static SimpleTargetServer INSTANCE = null;

	public static SimpleTargetServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new SimpleTargetServer();
		}

		return INSTANCE;
	}

	private static final Pose2d NO_POSE = new Pose2d();

	public static class PeriodicIO {
		public boolean targetValid;
		public Optional<TimestampedValue<Transform2d>> cameraToTarget;
	}

	private final StringPublisher m_statePublisher;
	private final PeriodicIO m_periodicIO;
	private static final String NAME = "";

	private SimpleTargetServer() {
		String nt_key = "target_server";

		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(nt_key);
		m_statePublisher = myTable.getStringTopic("state").publish();
		m_statePublisher.set("prestart");

		m_periodicIO = new PeriodicIO();
	}

	private static double measuredMetersToRealMeters(final double measured) {
		return 0.887 * measured + 0.038;
	}

	@Override
	public void periodic() {
		m_periodicIO.targetValid = LimelightHelpers.getTV(NAME);

		if (!m_periodicIO.targetValid) {
			m_statePublisher.set("no target");
			m_periodicIO.cameraToTarget = Optional.empty();
			return;
		}

		// find angle of targets in frame
		final var targetOffsetAngle = LIMELIGHT_PITCH.plus(Rotation2d.fromDegrees(LimelightHelpers.getTY(NAME)));
		final var distanceMeters = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT_METERS) / Math.tan(targetOffsetAngle.getRadians());
		final var realDistanceMeters = measuredMetersToRealMeters(distanceMeters);
		final var cameraToTarget = new Transform2d(realDistanceMeters, 0.0, Rotation2d.fromDegrees(-1.0 * LimelightHelpers.getTX(NAME)));
		m_periodicIO.cameraToTarget = Optional.of(new TimestampedValue<>(Timer.getFPGATimestamp(), cameraToTarget));
	}

	public Optional<Transform2d> getCameraToTarget() {
		return m_periodicIO.cameraToTarget.map(TimestampedValue::getValue);
	}
}
