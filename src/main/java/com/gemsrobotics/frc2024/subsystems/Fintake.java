package com.gemsrobotics.frc2024.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.frc2024.OI;
import com.gemsrobotics.lib.TalonUtils;
import com.gemsrobotics.lib.math.Units;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.math.MathUtil.isNear;

import java.util.Objects;

public class Fintake implements Subsystem {
	private static final String NT_KEY = "fintake";
	private static final boolean DO_STOPPING = true;

	private static final double DEPLOYMENT_RATIO = 25.48; // 25.48 turns of the motor spins the intake around once
	private static final double VELOCITY_RATIO = 1.7; // 1.7 turns of the motor spins the intake rollers once
	private static final double kV = 0.12 / 1.7;
	private static final double kG = -0.42; // volts

	private static final double INTAKE_SHOT_PROTECTION_SECONDS = 0.75;

	private static final double ROLLER_RADIUS = Units.inches2Meters(1.25) / 2.0;
	public static final double INTAKE_STARTING_ROTATIONS = -0.22;
	public static final double INTAKE_UNPRESS_ROTATIONS = -0.17;
	public static final double INTAKE_HORIZONTAL_ROTATIONS = 0.0;
	public static final double INTAKE_DEPLOYED_ROTATIONS = 0.14;

	private static Fintake INSTANCE;
	public static Fintake getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Fintake();
		}

		return INSTANCE;
	}

	public enum WantedState {
		NEUTRAL,
		INTAKING,
		SHOOTING,
		EXHAUSTING,
		INTAKING_AND_SHOOTING,
		OUT_AND_OFF
	}

	public enum State {
		EMPTY,
		INTAKING,
		HOLDING,
		SHOOTING,
		EXHAUSTING,
		INTAKING_AND_SHOOTING,
		OUT_AND_OFF
	}

	public static class PeriodicIO {
		public double intakeCurrentDrawAmps = 0.0;
		public double feederCurrentDrawAmps = 0.0;
		public double feederPosition = 0.0;
		public double deployerPosition = 0.0;
		public double deployerVoltsApplied = 0.0;
		public double deployerReference = 0.0;
		public double lastIntakeTimestamp = 0.0;
		public double nextReadyToShootTime = -1.0;
		public boolean hasBackedOut = true;
		public WantedState wantedState = WantedState.NEUTRAL;
		public State state = State.EMPTY;
	}

	private final TalonFX m_intake, m_feeder, m_deployer;
	private final StatusSignal<Double> m_intakeDrawAmpsSignal, m_feederDrawAmpsSignal, m_deployerPositionSignal, m_deployerVoltsApplied, m_deployerReference, m_feederPositionSignal;
	private final DoublePublisher m_intakeCurrentPublisher, m_feederCurrentPublisher, m_deployerPositionPublisher, m_deployerVoltsPublisher, m_deployerReferencePublisher;
	private final StringPublisher m_wantedStatePublisher, m_currentStatePublisher;
	private final DutyCycleOut m_intakeRequest;
	private final VoltageOut m_feederVoltsRequest;
	private final PositionTorqueCurrentFOC m_feederBackoutRequest;
	private final LinearFilter m_feederCurrentFilter;
	private final PeriodicIO m_periodicIO;
//	private final MotionMagicExpoVoltage m_deployerRequest;
	private final PositionVoltage m_deployerRequest;
	private boolean m_forcedOut;

	private final Timer m_noteGetTimer;

	private Fintake() {
		m_noteGetTimer = new Timer();
		m_noteGetTimer.stop();

		m_intake = new TalonFX(40, Constants.AUX_BUS_NAME);
		final var intakeConfig = new TalonFXConfiguration();
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		intakeConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 60.0;
		intakeConfig.Feedback.RotorToSensorRatio = 1.0;
		intakeConfig.Feedback.SensorToMechanismRatio = 1.0;
		intakeConfig.Slot0.kS = 0.05;
		intakeConfig.Slot0.kV = 0.12 * VELOCITY_RATIO;
		intakeConfig.Slot0.kA = 0.0;
		intakeConfig.Slot0.kP = 0.1;
		intakeConfig.Slot0.kI = 0.0;
		intakeConfig.Slot0.kD = 0.0;
		TalonUtils.configureTalon(intakeConfig, m_intake);
		m_intakeDrawAmpsSignal = m_intake.getTorqueCurrent();

		m_feeder = new TalonFX(23, Constants.AUX_BUS_NAME);
		final var feederConfig = new TalonFXConfiguration();
		feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		feederConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
		feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		feederConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		feederConfig.Feedback.RotorToSensorRatio = 1.0;
		feederConfig.Feedback.SensorToMechanismRatio = 1.0;
		feederConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
		feederConfig.Slot0.kS = 40.0;
		feederConfig.Slot0.kV = 0.0;
		feederConfig.Slot0.kA = 0.0;
		feederConfig.Slot0.kP = 50.0;
		feederConfig.Slot0.kI = 0.0;
		feederConfig.Slot0.kD = 0.0;
		TalonUtils.configureTalon(feederConfig, m_feeder);
		m_feederDrawAmpsSignal = m_feeder.getTorqueCurrent();
		m_feederPositionSignal = m_feeder.getPosition();

		m_deployer = new TalonFX(41, Constants.AUX_BUS_NAME);
		final var deployerConfig = new TalonFXConfiguration();
		deployerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		deployerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		deployerConfig.Feedback.RotorToSensorRatio = 1.0;
		deployerConfig.Feedback.SensorToMechanismRatio = DEPLOYMENT_RATIO;
		deployerConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		deployerConfig.Slot0.kG = kG;
		deployerConfig.Slot0.kP = 25.0;
		deployerConfig.Slot0.kI = 0.0;
		deployerConfig.Slot0.kD = 0.0;
		deployerConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
		deployerConfig.MotionMagic.MotionMagicExpo_kV = 0.01;
		deployerConfig.MotionMagic.MotionMagicExpo_kA = 0.001;
		TalonUtils.configureTalon(deployerConfig, m_deployer);
		m_deployer.setPosition(INTAKE_STARTING_ROTATIONS);
		m_deployerPositionSignal = m_deployer.getPosition();
		m_deployerVoltsApplied = m_deployer.getMotorVoltage();
		m_deployerReference = m_deployer.getClosedLoopReference();

		m_feederBackoutRequest = new PositionTorqueCurrentFOC(0.0);
		m_intakeRequest = new DutyCycleOut(0.0);
		m_intakeRequest.EnableFOC = false;
		m_feederVoltsRequest = new VoltageOut(0.0);
		m_feederVoltsRequest.EnableFOC = true;
		m_deployerRequest = new PositionVoltage(INTAKE_STARTING_ROTATIONS);
		m_deployerRequest.EnableFOC = true;

		m_forcedOut = false;
		m_feederCurrentFilter = LinearFilter.movingAverage(1);
		m_periodicIO = new PeriodicIO();

		// configure logging
		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
		m_intakeCurrentPublisher = myTable.getDoubleTopic("intake_current_draw").publish();
		m_feederCurrentPublisher = myTable.getDoubleTopic("feeder_current_draw").publish();
		m_deployerPositionPublisher = myTable.getDoubleTopic("deployer_position").publish();
		m_deployerVoltsPublisher = myTable.getDoubleTopic("deployer_applied_volts").publish();
		m_deployerReferencePublisher = myTable.getDoubleTopic("deployer_reference").publish();
		m_wantedStatePublisher = myTable.getStringTopic("state_wanted").publish();
		m_currentStatePublisher = myTable.getStringTopic("state_current").publish();
	}

	@Override
	public void periodic() {
		if (m_noteGetTimer.hasElapsed(.5)) {
			m_noteGetTimer.stop();
			m_noteGetTimer.reset();
			OI m_oi = OI.getInstance();
			m_oi.getPilot().setRumble(GenericHID.RumbleType.kBothRumble, 0.0); // 0 to 1
		}

		m_periodicIO.intakeCurrentDrawAmps = m_intakeDrawAmpsSignal.refresh().getValue();
		m_intakeCurrentPublisher.set(m_periodicIO.intakeCurrentDrawAmps);

		final double newFeederCurrent = m_feederDrawAmpsSignal.refresh().getValue();
		m_periodicIO.feederCurrentDrawAmps = m_feederCurrentFilter.calculate(newFeederCurrent);
		m_feederCurrentPublisher.set(m_periodicIO.feederCurrentDrawAmps);
		m_periodicIO.feederPosition = m_feederPositionSignal.refresh().getValue();

		m_periodicIO.deployerPosition = m_deployerPositionSignal.refresh().getValue();
		m_deployerPositionPublisher.set(m_periodicIO.deployerPosition);
		m_periodicIO.deployerVoltsApplied = m_deployerVoltsApplied.refresh().getValue();
		m_deployerVoltsPublisher.set(m_periodicIO.deployerVoltsApplied);
		m_periodicIO.deployerReference = m_deployerReference.refresh().getValue();
		m_deployerReferencePublisher.set(m_periodicIO.deployerReference);

		State newState = State.EMPTY;
		switch (m_periodicIO.wantedState) {
			case NEUTRAL:
				//immediately switch to neutral disregarding previous state.
				m_deployerRequest.Position = INTAKE_STARTING_ROTATIONS;

				m_intakeRequest.Output = 0.0;
				m_feederVoltsRequest.Output = 0.0;

				newState = switch (m_periodicIO.state) {
					case EMPTY, HOLDING -> m_periodicIO.state;
					case INTAKING, SHOOTING, EXHAUSTING, INTAKING_AND_SHOOTING, OUT_AND_OFF -> State.EMPTY;
				};

				break;

			case INTAKING:
				switch (m_periodicIO.state) {
					case EMPTY:
					case INTAKING:
						//if we want to intake and we are already intaking, keep at it.
						//if DO_STOPPING is true, then when the feeder is stalled set the state to HOLDING
						m_deployerRequest.Position = INTAKE_DEPLOYED_ROTATIONS;
						m_intakeRequest.Output = (DriverStation.isAutonomous() ? 10.0 : 8.0) / 12.0;
						m_feederVoltsRequest.Output = -7.0;
						newState = DO_STOPPING && isFeederStalled() ? State.HOLDING : State.INTAKING;

						if (newState == State.HOLDING) {
							LEDManager.getInstance().playNoteGetAnimation();
							if (DriverStation.isAutonomous()) {
								OI m_oi = OI.getInstance();
								m_oi.getPilot().setRumble(GenericHID.RumbleType.kBothRumble, 0.5); // 0 to 1
							}
							m_periodicIO.lastIntakeTimestamp = Timer.getFPGATimestamp();
							m_periodicIO.nextReadyToShootTime = m_periodicIO.lastIntakeTimestamp + INTAKE_SHOT_PROTECTION_SECONDS;
							m_periodicIO.hasBackedOut = false;
//							m_feederBackoutRequest.Position = m_feederPositionSignal.refresh().getValue() + 0.25; // backout one rotation
						}

						break;
					case HOLDING:
						// if we want to intake and we are holding a note keep holding it
						m_deployerRequest.Position = INTAKE_STARTING_ROTATIONS;
						m_intakeRequest.Output = 0.0;
						m_feederVoltsRequest.Output = 0.0;
						newState = State.HOLDING;
						break;

					case SHOOTING:
						//if we want to intake and we are shooting, keep shooting.
						m_deployerRequest.Position = INTAKE_STARTING_ROTATIONS;
						m_intakeRequest.Output = 0.0;
						m_feederVoltsRequest.Output = -11.0;
						newState = State.SHOOTING;
						break;

					case EXHAUSTING:
						//if we want to intake and we are exhausting, keep exhausting
						m_deployerRequest.Position = INTAKE_HORIZONTAL_ROTATIONS;
						m_intakeRequest.Output = -0.4;
						m_feederVoltsRequest.Output = -6.0;
						newState = State.EXHAUSTING;
						break;
				}

				break;

			case INTAKING_AND_SHOOTING:
				m_deployerRequest.Position = INTAKE_DEPLOYED_ROTATIONS;
				m_intakeRequest.Output = 7.0 / 12.0;
				m_feederVoltsRequest.Output = -11.0;
				newState = State.INTAKING_AND_SHOOTING;
				break;

			case SHOOTING:
				m_deployerRequest.Position = INTAKE_UNPRESS_ROTATIONS;
				m_intakeRequest.Output = 0.0;
				m_feederVoltsRequest.Output = -11.0;
				newState = State.SHOOTING;
				break;

			case EXHAUSTING:
				m_deployerRequest.Position = INTAKE_HORIZONTAL_ROTATIONS;
				m_intakeRequest.Output = -0.4;
				m_feederVoltsRequest.Output = 6.0;
				newState = State.EXHAUSTING;
				break;

			case OUT_AND_OFF:
				m_deployerRequest.Position = INTAKE_UNPRESS_ROTATIONS;
				m_intakeRequest.Output = 0.0;
				m_feederVoltsRequest.Output = 0.0;
				newState = State.OUT_AND_OFF;
				break;
		}

		m_currentStatePublisher.set(m_periodicIO.state.name());
		m_wantedStatePublisher.set(m_periodicIO.wantedState.name());

		m_intake.setControl(m_intakeRequest);

		final var timeHolding = Timer.getFPGATimestamp() - m_periodicIO.lastIntakeTimestamp;
		if (m_periodicIO.state == State.HOLDING && timeHolding > 0.25 && timeHolding < INTAKE_SHOT_PROTECTION_SECONDS) {
			if (!m_periodicIO.hasBackedOut) {
				m_feederBackoutRequest.Position = m_periodicIO.feederPosition + 0.45;
				m_periodicIO.hasBackedOut = true;
			}

			m_feeder.setControl(m_feederBackoutRequest);
		} else  {
			m_feeder.setControl(m_feederVoltsRequest);
		}

		if (m_forcedOut) {
			m_deployerRequest.Position = INTAKE_DEPLOYED_ROTATIONS;
		}

		m_deployer.setControl(m_deployerRequest);

		setState(newState);
	}

	public boolean isHoldingPiece() {
		return m_periodicIO.state == State.HOLDING;
	}

//	public void setIntakeReset() {
//		m_deployer.setPosition(INTAKE_STARTING_ROTATIONS);
//	}
//
//	private boolean m_stallIntakeBack = false;
//	public void setStallIntakeBack(final boolean s) {
//		m_stallIntakeBack = s;
//	}

	public void clearPiece() {
		setState(State.EMPTY);
	}

	public boolean isFeederStalled() {
		return m_periodicIO.feederCurrentDrawAmps < -35.0;
	}

	public void setWantedState(final WantedState state) {
		m_periodicIO.wantedState = state;
	}

	public double getReadyToShootTime() {
		return m_periodicIO.nextReadyToShootTime;
	}

	private synchronized void setState(final State newState) {
		m_periodicIO.state = newState;
	}

	public void setForcedOut(final boolean newForcedOut) {
		m_forcedOut = newForcedOut;
	}

	public State getState() {
		return m_periodicIO.state;
	}

	public double getLastIntakeTimestamp() {
		return m_periodicIO.lastIntakeTimestamp;
	}
}