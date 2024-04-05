package com.gemsrobotics.frc2024.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.lib.TalonUtils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

import static edu.wpi.first.math.MathUtil.isNear;

public final class Shooter implements Subsystem {
	private static final String NT_KEY = "shooter";

	private static final double SHOOTER_RATIO = 1.0 / 1.42; // 1 motor rotation is 1.7 shooter wheel rotations
	private static final double NOTE_DETECTED_AMPS = 30.0;
	private static final double IDLE_VELOCITY_RPS = 0.0;//2.0;

	private static Shooter INSTANCE;
	public static Shooter getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Shooter();
		}

		return INSTANCE;
	}

	private final TalonFX m_wheelLeft, m_wheelRight;
	private final PeriodicIO m_periodicIO;
	private final VelocityVoltage m_velocityRequest;
	private final PositionVoltage m_positionRequest;
	private final Follower m_followRequest;
	private final PositionTorqueCurrentFOC m_spitRequest;
	private final NeutralOut m_neutralRequest;
	private final StatusSignal<Double> m_velocityLeftSignal, m_velocityRightSignal;
	private final StatusSignal<Double> m_currentDrawLeft, m_currentDrawRight;
	private final StatusSignal<Double> m_positionLeft;
	private final StatusSignal<Double> m_voltageLeft, m_voltageRight;
	private final DoublePublisher m_velocityMeasuredLeftPublisher, m_velocityMeasuredRightPublisher;
	private final DoublePublisher m_velocityReferenceLeftPublisher, m_velocityReferenceRightPublisher;
	private final DoublePublisher m_currentLeftPublisher, m_currentRightPublisher;
	private final DoublePublisher m_voltageLeftPublisher, m_voltageRightPublisher;
	private final BooleanPublisher m_noteCaughtPublisher;
	private final StringPublisher m_statePublisher;
	private final MedianFilter m_velocityFilterLeft;
	private final MedianFilter m_velocityFilterRight;
	private final LinearFilter m_currentFilter;
	private final BaseStatusSignal[] m_baseSignals;
	private boolean m_doIdling;
	private final TorqueCurrentFOC m_idleRequest;

	public enum State {
		OFF,
		IDLE,
		SHOOTING,
		CATCHING_NOTE,
		SPITTING_NOTE
	}

	private static class PeriodicIO {
		public State state = State.IDLE;
		public boolean isNoteCaught = false;
		public double catchNoteGoal = 0.0;
		public double filteredVelocityLeftRPS = 0.0;
		public double filteredVelocityRightRPS = 0.0;
		public double referenceVelocityLeftRPS = 0.0;
		public double referenceVelocityRightRPS = 0.0;
	}

	private Shooter() {
		m_wheelLeft = new TalonFX(20, Constants.AUX_BUS_NAME);
		m_wheelRight = new TalonFX(21, Constants.AUX_BUS_NAME);

		m_periodicIO = new PeriodicIO();

		m_velocityFilterLeft = new MedianFilter(5);
		m_velocityFilterRight = new MedianFilter(5);
		m_currentFilter = LinearFilter.movingAverage(5);

		m_velocityRequest = new VelocityVoltage(0.0);
		m_velocityRequest.EnableFOC = true;
		m_velocityRequest.Slot = 0;

		m_positionRequest = new PositionVoltage(0.0);
		m_positionRequest.EnableFOC = true;
		m_positionRequest.Slot = 1;

		m_idleRequest = new TorqueCurrentFOC(20);

		m_followRequest = new Follower(m_wheelLeft.getDeviceID(), false);

		m_spitRequest = new PositionTorqueCurrentFOC(0.0); // !!!
		m_spitRequest.Slot = 2;

		m_neutralRequest = new NeutralOut();

		m_doIdling = false;

		//configure the two motors
		var wheelConfig = new TalonFXConfiguration();
		wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		wheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
		wheelConfig.Feedback.RotorToSensorRatio = 1.0;
		wheelConfig.Feedback.SensorToMechanismRatio = SHOOTER_RATIO;
		// velocity gains
		wheelConfig.Slot0.kP = 0.35;
		wheelConfig.Slot0.kI = 0.0;
		wheelConfig.Slot0.kD = 0.0;
		// Falcon 500 is a 500kV motor, 500rpm per V = 8.333 * 1.7 rps per V, 1/14.16 = 0.07 volts / Rotation per second
		// Falcon 500 is a 500kV motor, 500rpm per V = 8.333 * 1.42 rps per V, 1/11.8 = 0.0846 volts / Rotation per second
		wheelConfig.Slot0.kV = 0.0846;
		wheelConfig.Slot0.kS = 0.7;
		wheelConfig.Slot0.kA = 0.5;
		// positional gains
		wheelConfig.Slot1.kP = 3.0; // volts per rotation
		wheelConfig.Slot1.kI = 0.0;
		wheelConfig.Slot1.kD = 0.0;
		// positional gains (with a note in it)
//		wheelConfig.Slot2.kS = 90;
		wheelConfig.Slot2.kP = 200;
		wheelConfig.Slot2.kI = 0.0;
		wheelConfig.Slot2.kD = 0.0;

		wheelConfig.Voltage.PeakForwardVoltage = 12.0;
		wheelConfig.Voltage.PeakReverseVoltage = -12.0;
		wheelConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
		wheelConfig.CurrentLimits.SupplyCurrentLimit = 40; // lol
		wheelConfig.CurrentLimits.StatorCurrentLimitEnable = false;
		wheelConfig.CurrentLimits.StatorCurrentLimit = 200; // lol

		TalonUtils.configureTalon(wheelConfig, m_wheelLeft);
		// invert the other wheel
		wheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		TalonUtils.configureTalon(wheelConfig, m_wheelRight);

		// grab status signals to monitor velocity (RPS) and torque current (A)
		m_velocityLeftSignal = m_wheelLeft.getVelocity();
		m_velocityRightSignal = m_wheelRight.getVelocity();

		m_currentDrawLeft = m_wheelLeft.getTorqueCurrent();
		m_currentDrawRight = m_wheelRight.getTorqueCurrent();

		m_voltageLeft = m_wheelLeft.getMotorVoltage();
		m_voltageRight = m_wheelRight.getMotorVoltage();

		m_positionLeft = m_wheelLeft.getPosition();

		// configure logging of velocity (RPS) and torgue current (A)
		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
		m_velocityMeasuredLeftPublisher = myTable.getDoubleTopic("velocity_left_rps_measured").publish();
		m_velocityMeasuredRightPublisher = myTable.getDoubleTopic("velocity_right_rps_measured").publish();
		m_velocityReferenceLeftPublisher = myTable.getDoubleTopic("velocity_left_rps_reference").publish();
		m_velocityReferenceRightPublisher = myTable.getDoubleTopic("velocity_right_rps_reference").publish();
		m_currentLeftPublisher = myTable.getDoubleTopic("current_left_amps").publish();
		m_currentRightPublisher = myTable.getDoubleTopic("current_right_amps").publish();
		m_voltageLeftPublisher = myTable.getDoubleTopic("voltage_left").publish();
		m_voltageRightPublisher = myTable.getDoubleTopic("voltage_right").publish();
		m_noteCaughtPublisher = myTable.getBooleanTopic("note_caught").publish();
		m_statePublisher = myTable.getStringTopic("state").publish();

		m_baseSignals = new BaseStatusSignal[] {
			m_velocityLeftSignal,
			m_velocityRightSignal,
			m_currentDrawLeft,
		    m_currentDrawRight,
		    m_voltageLeft,
		    m_voltageRight,
			m_positionLeft
		};
	}

	private boolean accelerated = false;

	@Override
	public void periodic() {
		BaseStatusSignal.refreshAll(m_baseSignals);

		// telemetry
		final double leftVelocityRPS = m_velocityLeftSignal.getValue();
		final double rightVelocityRPS = m_velocityRightSignal.getValue();
		m_periodicIO.filteredVelocityLeftRPS = m_velocityFilterLeft.calculate(leftVelocityRPS);
		m_periodicIO.filteredVelocityRightRPS = m_velocityFilterRight.calculate(rightVelocityRPS);

		m_currentLeftPublisher.set(m_currentDrawLeft.getValue());
		m_currentRightPublisher.set(m_currentDrawRight.getValue());

		m_voltageLeftPublisher.set(m_voltageLeft.getValue());
		m_voltageRightPublisher.set(m_voltageRight.getValue());

		m_velocityMeasuredLeftPublisher.set(m_periodicIO.filteredVelocityLeftRPS);
		m_velocityMeasuredRightPublisher.set(m_periodicIO.filteredVelocityRightRPS);

		m_velocityReferenceLeftPublisher.set(m_periodicIO.referenceVelocityLeftRPS);
		m_velocityReferenceRightPublisher.set(m_periodicIO.referenceVelocityRightRPS);

		final double currentNow = m_currentFilter.calculate((m_currentDrawLeft.getValue() + m_currentDrawRight.getValue()) / 2.0);

		m_noteCaughtPublisher.set(m_periodicIO.isNoteCaught);
		m_statePublisher.set(m_periodicIO.state.toString());

		// write
		switch (m_periodicIO.state) {
			case OFF:
				m_periodicIO.isNoteCaught = false;

				if (DriverStation.isAutonomousEnabled() && m_doIdling) {
					m_wheelLeft.setControl(m_idleRequest);
					m_wheelRight.setControl(m_idleRequest);
				} else {
					m_wheelLeft.setControl(m_neutralRequest);
					m_wheelRight.setControl(m_neutralRequest);
				}
				break;
			case IDLE:
			case SHOOTING:
				m_periodicIO.isNoteCaught = false;
				m_wheelLeft.setControl(m_velocityRequest.withVelocity(m_periodicIO.referenceVelocityLeftRPS));
				m_wheelRight.setControl(m_velocityRequest.withVelocity(m_periodicIO.referenceVelocityRightRPS));
				break;
			case CATCHING_NOTE:
				if (m_periodicIO.isNoteCaught) {
					m_wheelLeft.setControl(m_positionRequest.withPosition(m_periodicIO.catchNoteGoal).withOverrideBrakeDurNeutral(true));
					m_wheelRight.setControl(m_followRequest);
				} else {
					m_wheelLeft.setControl(m_velocityRequest.withVelocity(5));
					m_wheelRight.setControl(m_velocityRequest.withVelocity(5));
				}

				break;
			case SPITTING_NOTE:
				m_wheelLeft.setControl(m_velocityRequest.withVelocity(10));
				m_wheelRight.setControl(m_velocityRequest.withVelocity(10));
				break;
		}
	}

	public double getMeasuredSpeed() {
		return m_periodicIO.filteredVelocityLeftRPS;
	}

	public double getMeasuredCurrentDraw() {
		return m_currentDrawLeft.getValue();
	}

	public boolean isNoteCaught() {
		return m_periodicIO.isNoteCaught;
	}

	public boolean isNoteHalfOutOrMore() {
		final var p = m_positionLeft.getValue();
		return isNear(p, m_periodicIO.catchNoteGoal, 0.25) || p > m_periodicIO.catchNoteGoal;
	}

	public boolean isReadyToShoot() {
		return m_periodicIO.state == State.SHOOTING
				&& isNear(m_periodicIO.referenceVelocityLeftRPS, m_periodicIO.filteredVelocityLeftRPS,3)
				&& isNear(m_periodicIO.referenceVelocityRightRPS, m_periodicIO.filteredVelocityRightRPS,3);
	}

	public void setVelocity(final double rpsLeft, final double rpsRight) {
		m_periodicIO.state = State.SHOOTING;
		m_periodicIO.referenceVelocityLeftRPS = rpsLeft;
		m_periodicIO.referenceVelocityRightRPS = rpsRight;
	}

	private static final double SPIN_RATIO = 55.0 / 85.0;
	public void setCurvedShot(final double fasterSpeedRps) {
		final var alliance = DriverStation.getAlliance();

		if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Red) {
			setVelocity(fasterSpeedRps * SPIN_RATIO, fasterSpeedRps);
		} else {
			setVelocity(fasterSpeedRps, fasterSpeedRps * SPIN_RATIO);
		}
	}

	public void setOff() {
		m_periodicIO.state = State.OFF;
		m_periodicIO.referenceVelocityLeftRPS = 0.0;
		m_periodicIO.referenceVelocityRightRPS = 0.0;
	}

	public void setIdle() {
		m_periodicIO.state = State.IDLE;
		m_periodicIO.referenceVelocityLeftRPS = IDLE_VELOCITY_RPS;
		m_periodicIO.referenceVelocityRightRPS = IDLE_VELOCITY_RPS;
	}

	public void setSpitting() {
		if (m_periodicIO.state != State.SPITTING_NOTE) {
			m_periodicIO.state = State.SPITTING_NOTE;
			m_spitRequest.Position = m_positionLeft.getValue() + 10.0;
		}
	}

	public void setCatchingNote() {
		if (m_periodicIO.state != State.CATCHING_NOTE) {
			m_periodicIO.state = State.CATCHING_NOTE;
			m_periodicIO.isNoteCaught = false;
			m_periodicIO.catchNoteGoal = Double.NEGATIVE_INFINITY;
			accelerated = false;
		}
	}

	public void setDoIdling(final boolean newDoIdling) {
		m_doIdling = newDoIdling;
	}
}
