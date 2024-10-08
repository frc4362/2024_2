package com.gemsrobotics.frc2024.subsystems;

import com.gemsrobotics.frc2024.*;
import com.gemsrobotics.frc2024.VisionServer;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import com.gemsrobotics.lib.allianceconstants.AllianceConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public final class Superstructure implements Subsystem {
	private static final boolean USE_DASHBOARD_SHOOTING = false;
	private static final boolean DO_SHOT_PROTECTION = true;

	private static final String NT_KEY = "superstructure";
	private static final String WANTED_STATE_KEY = "wanted_state";
	private static final String SYSTEM_STATE_KEY = "system_state";
	private static final String LOCALIZATION_STATE_KEY = "localization_state";
	private static final String SPEAKER_DISTANCE_KEY = "distance_speaker";
	private static final String AMP_ZONE_DISTANCE_KEY = "distance_amp_zone";
	private static final String MID_FEED_DISTANCE_KEY = "distance_mid_feed";
	private static final String SHOOTER_ANGLE_SET_KEY = "shooter_angle_degrees";
	private static final String SHOOTER_SPEED_SET_KEY = "shooter_set_rps";
	private static final String SHOT_PARAM_KEY = "shot_param";
	private static final String SHOOTING_STATE_KEY = "shooting_state";
	private static final String SHOT_TYPE_KEY = "shot_type";

	private static final double MAX_SHOOTING_SPEED_METERS_PER_SECOND = 0.5;

	public enum WantedState {
		IDLE,
		INTAKING,
		EXHAUSTING,
		SHOOTING,
		AMPING,
		CLIMBING,
		CLIMBING_2,
		PASSING,
		PRECLIMB
	}

	public enum SystemState {
		IDLE,
		INTAKING,
		EXHAUSTING,
		SHOOTING,
		AMPING,
		CLIMBING,
		CLIMBING_2,
		FLAT_PASSING,
		PRECLIMB
	}

	private static Superstructure INSTANCE = null;
	public static Superstructure getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Superstructure();
		}

		return INSTANCE;
	}

	private final Swerve m_drive;
	private final Shooter m_shooter;
	private final Fintake m_fintake;
	private final LEDManager m_leds;
	private final Timer m_stateChangedTimer;
	private final Timer m_autoClimbStartTimer;
	private final Arm m_arm;
	private final Bender m_bender;
	private final VisionServer m_targetServer;

	private final DoublePublisher m_speakerDistancePublisher, m_ampZoneDistancePublisher, m_midFeedDistancePublisher;
	private final StringPublisher m_wantedStatePublisher;
	private final StringPublisher m_systemStatePublisher;
	private final StringPublisher m_localizationStatePublisher;
	private final StringPublisher m_shotTypePublisher;
	private final StringPublisher m_shotParamPublisher;
	private final StringArrayPublisher m_shootingStatePublisher;

	private Optional<ShotParam> m_overrideShotParam;
	private AllianceConstants m_allianceConstants;
	private SystemState m_state;
	private WantedState m_stateWanted;
	private boolean m_isStrictlyLocalizing;
	private boolean m_stateChanged;
	private boolean m_wantsIntaking;
	private boolean m_wantsEarlyClimb;
	private boolean m_feedingAllowed;
	private double m_lastBadTagTime;
	private PeriodicIO m_periodicIO;
	
	private Superstructure() {
		m_arm = Arm.getInstance();
		m_drive = Swerve.getInstance();
		m_shooter = Shooter.getInstance();
		m_targetServer = VisionServer.getInstance();
		m_bender = Bender.getInstance();
		m_leds = LEDManager.getInstance();
		m_fintake = Fintake.getInstance();
		m_periodicIO = new PeriodicIO();

		m_stateChanged = true;
		m_isStrictlyLocalizing = true;
		m_wantsIntaking = false;
		m_wantsEarlyClimb = false;
		m_feedingAllowed = true;
		m_stateChangedTimer = new Timer();
		m_autoClimbStartTimer = new Timer();
		m_lastBadTagTime = -1_000.0;
		m_overrideShotParam = Optional.empty();

		m_allianceConstants = Constants.ALLIANCE_CONSTANTS_DEFAULT;

		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
		m_wantedStatePublisher = myTable.getStringTopic(WANTED_STATE_KEY).publish();
		m_systemStatePublisher = myTable.getStringTopic(SYSTEM_STATE_KEY).publish();
		m_localizationStatePublisher = myTable.getStringTopic(LOCALIZATION_STATE_KEY).publish();
		m_shotParamPublisher = myTable.getStringTopic(SHOT_PARAM_KEY).publish();
		m_speakerDistancePublisher = myTable.getDoubleTopic(SPEAKER_DISTANCE_KEY).publish();
		m_ampZoneDistancePublisher = myTable.getDoubleTopic(AMP_ZONE_DISTANCE_KEY).publish();
		m_midFeedDistancePublisher = myTable.getDoubleTopic(MID_FEED_DISTANCE_KEY).publish();
		m_shotTypePublisher = myTable.getStringTopic(SHOT_TYPE_KEY).publish();
		m_shootingStatePublisher = myTable.getStringArrayTopic(SHOOTING_STATE_KEY).publish();

		m_state = SystemState.IDLE;
		m_stateWanted = WantedState.IDLE;
	}

	public static class PeriodicIO {
		public Pose2d fieldToVehicle = new Pose2d();
		public double turnRate = 0.0;
	}

	@Override
	public void periodic() {
		m_periodicIO.fieldToVehicle = m_drive.getState().Pose;
		m_periodicIO.turnRate = m_drive.getPigeon2().getRate();

		final LocalizationState localizationState = updateLocalization();
		if (localizationState == LocalizationState.REJECTED_ON_DISTANCE) {
			m_lastBadTagTime = Timer.getFPGATimestamp();
		}

		// update telemetry
		final var shotType = getShotType();
		m_drive.setShotType(shotType);

		// telemetry
		m_systemStatePublisher.set(m_state.toString());
		m_wantedStatePublisher.set(m_stateWanted.toString());
		double distanceToGoal = getDistanceToSpeaker();
		m_speakerDistancePublisher.set(distanceToGoal);
		m_ampZoneDistancePublisher.set(getDistanceToAmpZone());
		m_midFeedDistancePublisher.set(getDistanceToMidFeed());
		m_shotTypePublisher.set(shotType.toString());
		m_shotParamPublisher.set(getDesiredShotParams().toString());

		if (OI.getInstance().getWantsNoteChase()
					&& m_fintake.isIntakeDown()
					&& NoteDetector.getInstance().getVehicleToNoteRotation().isPresent()
		) {
			m_leds.setLEDS(LEDManager.LEDstate.BAD_TAGS);
		} else if (DriverStation.isEnabled() &&
			(distanceToGoal > Constants.MAX_SUGGESTED_RANGE_METERS
				&& distanceToGoal < Constants.LED_SHUTOFF_RANGE_METERS)
		) {
			m_leds.setLEDS(LEDManager.LEDstate.OUT_OF_RANGE);
		} else if (localizationState == LocalizationState.LOCALIZED && m_isStrictlyLocalizing) {
			m_leds.setLEDS(LEDManager.LEDstate.IN_RANGE);
		} else {
			m_leds.setLEDS(LEDManager.LEDstate.IDLE);
		}

		final SystemState newState = switch (m_state) {
			case IDLE -> handleIdle();
			case INTAKING -> handleIntaking();
			case SHOOTING -> handleShooting();
			case EXHAUSTING -> handleExhausting();
			case AMPING -> handleAmping();
			case CLIMBING -> handleClimbing();
			case CLIMBING_2 -> handleClimbing_2();
			case FLAT_PASSING -> handlePassing();
			case PRECLIMB -> handlePreclimb();
			default -> SystemState.IDLE;
		};

		if (newState != m_state) {
			m_state = newState;
			m_stateChangedTimer.reset();
			m_stateChangedTimer.start();
			m_stateChanged = true;
		} else {
			m_stateChanged = false;
		}
	}

	private SystemState applyWantedState() {
		return switch (m_stateWanted) {
			case IDLE -> SystemState.IDLE;
			case INTAKING -> SystemState.INTAKING;
			case SHOOTING -> SystemState.SHOOTING;
			case AMPING -> SystemState.AMPING;
			case EXHAUSTING -> SystemState.EXHAUSTING;
			case CLIMBING -> SystemState.CLIMBING;
			case CLIMBING_2 -> SystemState.CLIMBING_2;
			case PASSING -> SystemState.FLAT_PASSING;
			case PRECLIMB -> SystemState.PRECLIMB;
		};
	}

	private enum LocalizationState {
		NO_TARGET,
		TURNING_TOO_FAST,
		LOCALIZED,
		REJECTED_ON_DISTANCE
	}

	private LocalizationState updateLocalization() {
		LimelightHelpers.SetRobotOrientation(
				"",
				m_drive.getState().Pose.getRotation().getDegrees(),
				0.0,
				0, 0, 0, 0);

		// supply vision measurements to robot
		// coordinate subsystems
		// act on state
		final var visionUpdate = m_targetServer.getLatestEstimate();
		if (visionUpdate.isPresent()) {
			final var update = visionUpdate.get();
			// MEGATAG 1
			if (m_isStrictlyLocalizing) {
				final var mt = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
				if (mt.pose.getTranslation().getNorm() == 0.0) {
					m_localizationStatePublisher.set("no targets detected (no update)");
					return LocalizationState.NO_TARGET;
				}

				m_drive.addVisionMeasurement(mt.pose, mt.timestampSeconds, VecBuilder.fill(.6,.6,0.0));
				m_localizationStatePublisher.set("strictly localized");
			// MEGATAG 2
			} else {
				final double yawRate = m_drive.getPigeon2().getRate();
				if (Math.abs(yawRate) > 720) {
					m_localizationStatePublisher.set("turning too fast");
					return LocalizationState.TURNING_TOO_FAST;
				}

				m_drive.addVisionMeasurement(update.pose, update.timestampSeconds, VecBuilder.fill(.3,.3,9_999_999));
				m_localizationStatePublisher.set("localized");
			}

			return LocalizationState.LOCALIZED;
		} else {
			m_localizationStatePublisher.set("no targets detected (no update)");
			return LocalizationState.NO_TARGET;
		}
	}

	private SystemState handleShooting() {
		m_bender.setWantedState(Bender.State.STOWED);

		final var params = getDesiredShotParams();
		if (DO_SHOT_PROTECTION && !m_wantsIntaking) {
			if (Timer.getFPGATimestamp() > m_fintake.getReadyToShootTime()) {
				conformToShooterState(params);
			}
		} else {
			conformToShooterState(params);
		}

		if (m_wantsIntaking) {
			m_fintake.setWantedState(Fintake.WantedState.INTAKING_AND_SHOOTING);
		} else if (isReadyToShoot(true) && m_feedingAllowed) {
			m_fintake.clearPiece();
			m_fintake.setWantedState(Fintake.WantedState.SHOOTING);
		} else {
			m_fintake.setWantedState(Fintake.WantedState.OUT_AND_OFF);
		}

		return applyWantedState();
	}

	private SystemState handleIntaking() {
		if (m_stateChanged) {
			m_fintake.clearPiece();
		}

		m_fintake.setWantedState(Fintake.WantedState.INTAKING);
		m_bender.setWantedState(Bender.State.STOWED);
		m_arm.setWantedState(Arm.State.STOWED);

		if (Timer.getFPGATimestamp() > m_fintake.getReadyToShootTime()) {
			m_shooter.setOff();
		} else {
			m_shooter.setVelocity(-2.5, -2.5);
		}

		return applyWantedState();
	}

	private boolean m_lockClimb = false;
	private SystemState handlePreclimb() {
		m_arm.setWantedState(Arm.State.STOWED);
		if (m_wantsEarlyClimb || m_lockClimb) {
			Climber.getInstance().setShortClimb();
			m_lockClimb = true;
		} else {
			Climber.getInstance().setExtended();
		}

		return applyWantedState();
	}

	private SystemState handleClimbing() {
		m_fintake.setForcedOut(true);
		m_fintake.setWantedState(Fintake.WantedState.OUT_AND_OFF);
		Climber.getInstance().setExtended();

		if (m_stateWanted == WantedState.CLIMBING) {
			m_arm.setTrapPose();
		} else {
			m_arm.setWantedState(Arm.State.STOWED);
		}

		if (m_arm.isTrapExtended()) {
			return SystemState.CLIMBING_2;
		// magic number
		} if (m_arm.getShoulderRotation() > -0.04 && m_stateWanted != WantedState.CLIMBING_2) {
			return SystemState.CLIMBING;
		} else {
			return applyWantedState();
		}
	}

	private final Timer m_climbDoneTimer = new Timer();

	private SystemState handleClimbing_2() {
		if (m_stateChanged) {
			m_climbDoneTimer.reset();
			m_autoClimbStartTimer.start();
		}

		if (m_stateChangedTimer.get() < 0.25) {
			return SystemState.CLIMBING_2;
		}

		Climber.getInstance().setRetracted();
		m_fintake.setForcedOut(true);
		m_fintake.setWantedState(Fintake.WantedState.OUT_AND_OFF);
		m_arm.setTrapPose();
		m_bender.setWantedState(Bender.State.TRAPPING);

		if (Climber.getInstance().isRetracted()) {
			if (m_climbDoneTimer.get() == 0.0) {
				m_climbDoneTimer.start();
				m_shooter.setVelocity(7.0, 7.0);
			} else if (m_climbDoneTimer.get() > 0.25) {
				m_fintake.setWantedState(Fintake.WantedState.AMPING);
			}
		} else {
			m_fintake.setWantedState(Fintake.WantedState.OUT_AND_OFF);
			m_shooter.setOff();
		}

		return SystemState.CLIMBING_2;
	}

	private SystemState handleExhausting() {
		m_fintake.setWantedState(Fintake.WantedState.EXHAUSTING);
		return applyWantedState();
	}

	private SystemState handleIdle() {
		m_fintake.setWantedState(Fintake.WantedState.NEUTRAL);
		final var lastPickupTimeD = (Timer.getFPGATimestamp() - m_fintake.getLastIntakeTimestamp());
		if (lastPickupTimeD > 0 && lastPickupTimeD < 2) {
			m_shooter.setVelocity(-2.5, -2.5);
		} else {
			m_shooter.setOff();
		}
		m_arm.setWantedState(Arm.State.STOWED);
		m_bender.setWantedState(Bender.State.STOWED);

		return applyWantedState();
	}

	private SystemState handleAmping() {
		if (m_stateChangedTimer.get() > 0.3) {
			m_shooter.setCatchingNote();
		}

		m_fintake.setWantedState(Fintake.WantedState.AMPING);

		if (m_stateWanted != WantedState.AMPING) {
			m_arm.setWantedState(Arm.State.STOWED);
			m_bender.setWantedState(Bender.State.STOWED);
		} else if (m_stateChangedTimer.get() > 0.3) {
			m_arm.setWantedState(Arm.State.AMP);
			m_bender.setWantedState(Bender.State.AMPING);
		}

		// magic number
		if (m_arm.getShoulderRotation() > -0.04) {
			return SystemState.AMPING;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handlePassing() {
		m_arm.setWantedState(Arm.State.FLAT);
		m_bender.setWantedState(Bender.State.STOWED);

		if (Timer.getFPGATimestamp() > m_fintake.getReadyToShootTime()) {
			m_shooter.setVelocity(90.0, 90.0);
		} else {
			m_shooter.setOff();
		}

		if (m_shooter.isReadyToShoot() && m_feedingAllowed) {
			m_fintake.clearPiece();
			m_fintake.setWantedState(Fintake.WantedState.SHOOTING);
		} else {
			m_fintake.setWantedState(Fintake.WantedState.NEUTRAL);
		}

		return applyWantedState();
	}

	public boolean isReadyToShoot(final boolean waitForStopRotating) {
		final boolean stopped = m_drive.getObserver().getMeasuredVelocity().getNorm() < MAX_SHOOTING_SPEED_METERS_PER_SECOND;
		final boolean shooterAtSpeed = m_shooter.isReadyToShoot();
		final boolean turningFast = !waitForStopRotating || Math.abs(Math.toRadians(m_periodicIO.turnRate)) < 1;
		final boolean armElevationCorrect = m_arm.atReference(getDesiredShotParams());

		final List<String> reasons = new ArrayList<>();

		if (!stopped) {
			reasons.add("not stopped");
		}

		if (!shooterAtSpeed) {
			reasons.add("shooter not at speed");
		}

		if (!turningFast) {
			reasons.add("turning too fast");
		}

		if (!armElevationCorrect) {
			reasons.add("elbow elevation incorrect");
		}

		m_shootingStatePublisher.set(reasons.toArray(new String[0]));

		return stopped && shooterAtSpeed && turningFast && armElevationCorrect;
	}

	private double getDistanceToSpeaker() {
		return m_allianceConstants.getSpeakerLocationMeters().getDistance(m_periodicIO.fieldToVehicle.getTranslation());
	}

	private double getDistanceToAmpZone() {
		return m_allianceConstants.getAmpZoneLocationMeters().getDistance(m_periodicIO.fieldToVehicle.getTranslation());
	}

	private double getDistanceToMidFeed() {
		return m_allianceConstants.getMiddleFeedLocationMeters().getDistance(m_periodicIO.fieldToVehicle.getTranslation());
	}

	private ShotParam getDashboardShooterParams() {
		return new ShotParam(
				Rotation2d.fromDegrees(SmartDashboard.getNumber(SHOOTER_ANGLE_SET_KEY, 0.0)),
				SmartDashboard.getNumber(SHOOTER_SPEED_SET_KEY, 0.0));
	}

	public ShotType getShotType() {
		final double speakerRange = getDistanceToSpeaker();

		if (speakerRange < 6.0) {
			return ShotType.SPEAKER;
		} else if (m_allianceConstants.isLocationInOpposingWing(m_periodicIO.fieldToVehicle.getTranslation())) {
			return ShotType.MID_FEED;
		} else {
			return ShotType.AMP_FEED;
		}
	}

	private ShotParam getVisionShooterParam() {
		final var shotType = getShotType();

		if (shotType == ShotType.SPEAKER) {
			return Constants.getShotParameters(getDistanceToSpeaker());
		}

		final double feedRange;
		if (shotType == ShotType.MID_FEED) {
			feedRange = getDistanceToMidFeed();
		} else {
			feedRange = getDistanceToAmpZone();
		}

		return Constants.getFeedParameters(feedRange);
	}

	public void setAlliance() {
		m_allianceConstants = Constants.getAllianceConstants();
		m_drive.setAllianceConstants(m_allianceConstants);
	}

	public void setStrictLocalizationEnabled(final boolean b) {
		m_isStrictlyLocalizing = b;
	}

	private void conformToShooterState(final ShotParam p) {
		if (Timer.getFPGATimestamp() > m_fintake.getReadyToShootTime()) {
			m_shooter.setCurvedShot(p.getVelocityRps());
		} else {
			m_shooter.setOff();
		}

		m_arm.setShootingAngle(p);
	}

	public void setWantedState(final WantedState state) {
		m_stateWanted = state;
	}

	public void setWantsIntaking(final boolean newIntaking) {
		m_wantsIntaking = newIntaking;
	}

	public void setOverrideShotParams(final ShotParam param) {
		m_overrideShotParam = Optional.of(param);
	}

	public void setOverrideParamsCleared() {
		m_overrideShotParam = Optional.empty();
	}

	public ShotParam getDesiredShotParams() {
		if (USE_DASHBOARD_SHOOTING) {
			return getDashboardShooterParams();
		}

		return m_overrideShotParam.orElseGet(this::getVisionShooterParam);
	}

	public void setFeedingAllowed(final boolean feedingAllowed) {
		m_feedingAllowed = feedingAllowed;
	}

	public void setWantsEarlyClimb(final boolean earlyClimb) {
		m_wantsEarlyClimb = earlyClimb;
	}

	public SystemState getState() {
		return m_state;
	}
}
