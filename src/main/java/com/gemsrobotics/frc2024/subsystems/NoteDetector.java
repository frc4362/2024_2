package com.gemsrobotics.frc2024.subsystems;

import com.gemsrobotics.frc2024.LimelightHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import java.util.Optional;

public class NoteDetector implements Subsystem {
	private static NoteDetector INSTANCE = null;

	public static NoteDetector getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new NoteDetector();
		}

		return INSTANCE;
	}

	private static final String LIMELIGHT_NAME = "limelight-drive";

	private static class PeriodicIO {
		public Optional<Rotation2d> vehicleToNoteRotation = Optional.empty();
	}

	private final PeriodicIO m_periodicIO;

	private NoteDetector() {
		m_periodicIO = new PeriodicIO();
	}

	@Override
	public void periodic() {
		if (!Fintake.getInstance().isIntakeDown()) {
			m_periodicIO.vehicleToNoteRotation = Optional.empty();
			return;
		}

		final double rot = LimelightHelpers.getTX(LIMELIGHT_NAME);
		if (rot != 0.0) {
			m_periodicIO.vehicleToNoteRotation = Optional.of(Rotation2d.fromDegrees(rot).unaryMinus());
		} else {
			m_periodicIO.vehicleToNoteRotation = Optional.empty();
		}
	}

	public Optional<Rotation2d> getVehicleToNoteRotation() {
		return m_periodicIO.vehicleToNoteRotation;
	}
}
