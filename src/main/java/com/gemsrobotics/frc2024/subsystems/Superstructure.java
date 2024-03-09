package com.gemsrobotics.frc2024.subsystems;

import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.frc2024.NewTargetServer;
import com.gemsrobotics.frc2024.ShotParam;
import com.gemsrobotics.frc2024.SimpleTargetServer;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import com.gemsrobotics.lib.allianceconstants.AllianceConstants;
import com.gemsrobotics.lib.data.Pose2dRollingAverage;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
	private static final String GOAL_DISTANCE_KEY = "goal_distance";
	private static final String NT_KEY = "superstructure";
	private static final String WANTED_STATE_KEY = "wanted_state";
	private static final String SYSTEM_STATE_KEY = "system_state";
	private static final String LOCALIZATION_STATE_KEY = "localization_state";
	public static final double MAX_SHOOTING_SPEED_METERS_PER_SECOND = 0.25;

	public enum WantedState {
		IDLE,
		INTAKING,
		EXHAUSTING,
		SHOOTING,
		AMPING,
		CLIMBING,
		CLIMBING_2
	}

	public enum SystemState {
		IDLE,
		INTAKING,
		EXHAUSTING,
		SHOOTING,
		AMPING,
		CLIMBING,
		CLIMBING_2
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
	private final Arm m_arm;
	private final Bender m_bender;
	private final NewTargetServer m_targetServer;
	private final SimpleTargetServer m_simpleTargetServer;

	private final DoublePublisher m_goalDistancePublisher;
	private final StringPublisher m_wantedStatePublisher;
	private final StringPublisher m_systemStatePublisher;
	private final StringPublisher m_localizationStatePublisher;
	private final StringPublisher m_shotParamPublisher;
	private final StringArrayPublisher m_shootingStatePublisher;

	private Optional<ShotParam> m_overrideShotParam;
	private AllianceConstants m_allianceConstants;
	private SystemState m_state;
	private WantedState m_stateWanted;
	private boolean m_isStrictlyLocalizing;
	private boolean m_stateChanged;
	private boolean m_ampCanSpit;
	private boolean m_wantsIntaking;
	private boolean m_shootWhileTurningAllowed;
	private boolean m_wantsFallbackShotRanging;
	private boolean m_feedingAllowed;
	private double m_lastBadTagTime;
	private int m_tooFarVisionReadingCount;
	private ShotLogger m_logger;
	
	private Superstructure() {
		m_arm = Arm.getInstance();
		m_drive = Swerve.getInstance();
		m_shooter = Shooter.getInstance();
		m_targetServer = NewTargetServer.getInstance();
		m_bender = Bender.getInstance();
		m_leds = LEDManager.getInstance();
		m_fintake = Fintake.getInstance();
		m_simpleTargetServer = SimpleTargetServer.getInstance();
		m_logger = new ShotLogger();

		m_stateChanged = true;
		m_isStrictlyLocalizing = true;
		m_ampCanSpit = false;
		m_wantsIntaking = false;
		m_shootWhileTurningAllowed = false;
		m_feedingAllowed = true;
		m_stateChangedTimer = new Timer();
		m_tooFarVisionReadingCount = 0;
		m_lastBadTagTime = -1_000.0;
		m_overrideShotParam = Optional.empty();

		m_allianceConstants = Constants.ALLIANCE_CONSTANTS_DEFAULT;

		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
		m_wantedStatePublisher = myTable.getStringTopic(WANTED_STATE_KEY).publish();
		m_systemStatePublisher = myTable.getStringTopic(SYSTEM_STATE_KEY).publish();
		m_localizationStatePublisher = myTable.getStringTopic(LOCALIZATION_STATE_KEY).publish();
		m_shotParamPublisher = myTable.getStringTopic("shot_param").publish();
		m_goalDistancePublisher = myTable.getDoubleTopic(GOAL_DISTANCE_KEY).publish();
		m_shootingStatePublisher = myTable.getStringArrayTopic("shooting_state").publish();

		m_state = SystemState.IDLE;
		m_stateWanted = WantedState.IDLE;
	}

	@Override
	public void periodic() {
		// telemetry
		m_systemStatePublisher.set(m_state.toString());
		m_wantedStatePublisher.set(m_stateWanted.toString());
		m_goalDistancePublisher.set(getDistanceToGoal());
		m_shotParamPublisher.set(getDesiredShotParams().toString());

		final LocalizationState localizationState = updateLocalization();
		if (localizationState == LocalizationState.REJECTED_ON_DISTANCE) {
			m_lastBadTagTime = Timer.getFPGATimestamp();
		}

		// blink for a second after a bad reading
		if ((Timer.getFPGATimestamp() - m_lastBadTagTime) < 0.1) {
			m_leds.setLEDS(LEDManager.LEDstate.BAD_TAGS);
		} else if (DriverStation.isEnabled() &&
			((getDistanceToGoal() > Constants.MAX_SUGGESTED_RANGE_METERS
				&& getDistanceToGoal() < Constants.LED_SHUTOFF_RANGE_METERS)
				|| getDistanceToGoal() < 2.1)
		) {
			m_leds.setLEDS(LEDManager.LEDstate.OUT_OF_RANGE);
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
		};
	}

	private enum LocalizationState {
		NO_TARGET,
		TURNING_TOO_FAST,
		LOCALIZED,
		REJECTED_ON_DISTANCE
	}

	private LocalizationState updateLocalization() {
		// supply vision measurements to robot
		// coordinate subsystems
		// act on state
		final var visionUpdate = m_targetServer.getLatestEstimate();
		if (visionUpdate.isPresent()) {
			final var update = visionUpdate.get();
			final var newPose = update.pose;

			final var distance = newPose.getTranslation().getDistance(m_drive.getState().Pose.getTranslation());

			if (Math.abs(Math.toRadians(m_drive.getPigeon2().getRate())) > 1.0) {
				m_localizationStatePublisher.set("turning too fast");
				return LocalizationState.TURNING_TOO_FAST;
			}

			// if the new update is close enough to our estimate
			if (m_isStrictlyLocalizing || distance < 5.0) {
				m_tooFarVisionReadingCount = 0;

				final var headingStd = m_isStrictlyLocalizing ? 0 : 999_999;

				m_drive.addVisionMeasurement(newPose, update.timestampSeconds, VecBuilder.fill(0.3, 0.3, headingStd));

				m_localizationStatePublisher.set(m_isStrictlyLocalizing ? "strictly localized" : "localized");
				return LocalizationState.LOCALIZED;
			} else {
				m_tooFarVisionReadingCount += 1;
				m_localizationStatePublisher.set("rejected on distance");
				return LocalizationState.REJECTED_ON_DISTANCE;
			}
		} else {
			m_localizationStatePublisher.set("no targets detected (no update)");
			return LocalizationState.NO_TARGET;
		}
	}

	private boolean m_hasLoggedShot = true;
	private SystemState handleShooting() {
		if (m_stateChanged) {
			m_hasLoggedShot = false;
		}

		m_bender.setWantedState(Bender.State.STOWED);

		final var params = getDesiredShotParams();
		conformToShooterState(params);

		if (m_wantsIntaking) {
			m_fintake.setWantedState(Fintake.WantedState.INTAKING_AND_SHOOTING);
		} else if (isReadyToShoot(true) && m_feedingAllowed) {
			if (!m_hasLoggedShot) {
				m_logger.logShot(params, m_shooter.getMeasuredSpeed(), m_arm.getElbowAngle(), getDistanceToGoal());
				m_hasLoggedShot = true;
			}

			m_fintake.clearPiece();
			m_fintake.setWantedState(Fintake.WantedState.SHOOTING);
		} else {
			m_fintake.setWantedState(Fintake.WantedState.NEUTRAL);
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
		m_shooter.setVelocity(-2.5, -2.5);
//		m_ledManager.setLEDS(LEDstate.OFF);
		return applyWantedState();
	}

	private SystemState handleClimbing() {
//		m_shooter.setCatchingNote();
		if (m_shooter.isNoteCaught()) {
			m_fintake.setWantedState(Fintake.WantedState.NEUTRAL);
		} else {
//			m_fintake.setWantedState(Fintake.WantedState.SHOOTING);
		}

		m_arm.setWantedState(Arm.State.CLIMB_PLACE);
		return applyWantedState();
	}

	private SystemState handleClimbing_2() {
		m_fintake.setWantedState(Fintake.WantedState.OUT_AND_OFF);
		m_arm.setFinalClimb();
//		m_arm.setWantedState(Arm.State.CLIMB_PLACE_2);
//
//		if (m_ampCanSpit) {
////			m_shooter.setSpitting();
//		} else {
////			m_shooter.setCatchingNote();
//		}
//
//		// TODO? arm ref measurement incorrect?
//		if (m_stateChangedTimer.get() > 2.0) {
//			m_bender.setWantedState(Bender.State.DEPLOYED);
//		} else {
//			m_bender.setWantedState(Bender.State.STOWED);
//		}

		return applyWantedState();
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
//		m_ledManager.setLEDS(LEDstate.OFF);
		return applyWantedState();
	}

	private boolean m_hasTriedToSpit = false;
	private SystemState handleAmping() {
		if (m_stateChanged) {
			m_hasTriedToSpit = false;
		}

		if (m_ampCanSpit || m_hasTriedToSpit) {
			m_shooter.setSpitting();
			m_hasTriedToSpit = true;
		} else {
			m_shooter.setCatchingNote();
		}

		if (m_stateChangedTimer.get() > 0.5) {
			m_fintake.setWantedState(Fintake.WantedState.SHOOTING);
		} else {
			m_fintake.setWantedState(Fintake.WantedState.NEUTRAL);
		}

		if (m_shooter.isNoteCaught() && m_shooter.isNoteHalfOutOrMore()) {
			m_bender.setWantedState(Bender.State.WIGGLING);
		} else {
			m_bender.setWantedState(Bender.State.STOWED);
		}

		m_arm.setWantedState(Arm.State.AMP);
		return applyWantedState();
	}

	private boolean isReadyToShoot(final boolean waitForStopRotating) {
		final boolean stopped = m_drive.getObserver().getMeasuredVelocity().getNorm() < MAX_SHOOTING_SPEED_METERS_PER_SECOND;
		final boolean shooterAtSpeed = m_shooter.isReadyToShoot();
		final boolean turningFast = !waitForStopRotating || Math.abs(m_drive.getPigeon2().getRate()) < 1;
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

	private double getDistanceToGoal() {
		return m_allianceConstants.getSpeakerLocationMeters().getDistance(m_drive.getState().Pose.getTranslation());
//		final var cameraToTarget = m_simpleTargetServer.getCameraToTarget();
//		return cameraToTarget.map(measure -> measure.getTranslation().getNorm()).orElse(0.0);
	}

	private static final PIDController TARGET_PID = new PIDController(0.3, 0.0, 0.2);
	static {
		TARGET_PID.setTolerance(Rotation2d.fromDegrees(1.).getRadians());
	}

	private ShotParam getDashboardShooterParams() {
		return new ShotParam(
				Rotation2d.fromDegrees(SmartDashboard.getNumber("shooter_angle_degrees", 0.0)),
				SmartDashboard.getNumber("shooter_set_rps", 0.0));
	}

	private ShotParam getVisionShooterParam() {
		return Constants.getShotParameters(getDistanceToGoal());
	}

	public Optional<Double> getGoalTurnFeedback() {
		return m_targetServer.getFallbackCameraToTarget().map(tf -> TARGET_PID.calculate(-tf.getRotation().getRadians()));
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

	public void setWantsAmpSpit(final boolean newWantsAmpSpit) {
		m_ampCanSpit = newWantsAmpSpit;
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

	public void setShootWhileTurning(final boolean newTurningAllowed) {
		m_shootWhileTurningAllowed = newTurningAllowed;
	}

	public void setFeedingAllowed(final boolean feedingAllowed) {
		m_feedingAllowed = feedingAllowed;
	}

	public void requestFallbackShotRanging() {
		m_wantsFallbackShotRanging = true;
	}

	public boolean isUsingFallbackAiming() {
		return m_wantsFallbackShotRanging;
	}

	public SystemState getState() {
		return m_state;
	}
}
