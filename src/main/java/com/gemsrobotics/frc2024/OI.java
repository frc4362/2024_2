package com.gemsrobotics.frc2024;

import com.gemsrobotics.lib.allianceconstants.AllianceConstants;
import com.gemsrobotics.lib.allianceconstants.AllianceConstantsBlue;
import com.gemsrobotics.lib.allianceconstants.AllianceConstantsRed;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

import java.util.Objects;
import java.util.Optional;

public final class OI {
	private static final boolean DO_EVADING = false;

	private static OI INSTANCE = null;

	public static OI getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new OI();
		}

		return INSTANCE;
	}

	private final XboxController m_pilot;
	private final XboxController m_copilot;

	private OI() {
		m_pilot = new XboxController(Constants.PILOT_JOYSTICK_PORT);
		m_copilot = new XboxController(Constants.COPILOT_JOYSTICK_PORT);
	}

	public Translation2d getWantedSwerveTranslation() {
		return new Translation2d(-m_pilot.getLeftY(), -m_pilot.getLeftX());
	}

	public double getWantedSwerveRotation() {
		return -m_pilot.getRightX();
	}

	public boolean getWantsEvasion() {
		return DO_EVADING && m_pilot.getLeftBumperPressed();
	}

//	public boolean getWantsLeftStageControl() {
//		return m_pilot.getRightTriggerAxis() > 0.7;
//	}
//
//	public boolean getWantsRightStageControl() {
//		return m_pilot.getRightTriggerAxis() > 0.7;
//	}

//	public Optional<Rotation2d> getSnapHeadingGoal() {
//		if (getWantsLeftStageControl()) {
//			return Optional.of(Rotation2d.fromDegrees(-30));
//		} else {
//			return Optional.empty();
//		}
//	}

	public boolean getWantsAiming() {
		return m_pilot.getRightBumper();
	}

	public XboxController getPilot() {
		return m_pilot;
	}

	public XboxController getCopilot() {
		return m_copilot;
	}
}
