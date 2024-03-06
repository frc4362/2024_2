package com.gemsrobotics.frc2024.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.lib.TalonUtils;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public class Climber implements Subsystem {

    private static final String NT_KEY = "climber";

    private static Climber INSTANCE;
    public static Climber getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new Climber();
        }
        return INSTANCE;
    }

    private final TalonFX m_climberBreaker, m_climberRio;
    private final PeriodicIO m_periodicIO;

    private final StatusSignal<Double>
            m_rotationClimberBreakerSignal, m_currentDrawClimberBreaker, m_rotationClimberRioSignal, m_currentDrawClimberRioSignal;
    private final DoublePublisher
            m_rotationClimberLeftPublisher, m_currentDrawClimberLeftPublisher, m_rotationClimberRightPublisher, m_currentDrawClimberRightPublisher;

    public static class PeriodicIO {
        public State wantedState = State.STOWED;
        public double rotationClimberLeftRotations = 0.0;
        public double currentDrawClimberLeftAmps = 0.0;
        public double rotationClimberRightRotations = 0.0;
        public double currentDrawClimberRightAmps = 0.0;
    }

    public enum State {
        STOWED,
        RETRACTED
    }

    private NeutralOut m_neutralOut;
    private PositionVoltage m_retractedPositionVoltage;
    private Follower m_followerRequest;

    private Climber() {
        m_climberBreaker = new TalonFX(17, Constants.AUX_BUS_NAME);
        m_climberRio = new TalonFX(16, Constants.AUX_BUS_NAME);

        m_periodicIO = new PeriodicIO();

        final var climberConfig = new TalonFXConfiguration();
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climberConfig.Feedback.RotorToSensorRatio = 1.0;
        climberConfig.Feedback.SensorToMechanismRatio = 1.0;
        TalonUtils.configureTalon(climberConfig, m_climberBreaker);
        TalonUtils.configureTalon(climberConfig, m_climberRio);

        // grab status signals to monitor velocity (RPS) and torque current (A)
        m_rotationClimberBreakerSignal = m_climberBreaker.getPosition();
        m_currentDrawClimberBreaker = m_climberBreaker.getTorqueCurrent();
        m_rotationClimberRioSignal = m_climberRio.getPosition();
        m_currentDrawClimberRioSignal = m_climberRio.getTorqueCurrent();

        // configure logging of velocity (RPS) and torque current (A)
        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
        m_rotationClimberLeftPublisher = myTable.getDoubleTopic("rotation_climber_left_rotations").publish();
        m_currentDrawClimberLeftPublisher = myTable.getDoubleTopic("current_climber_left_amps").publish();
        m_rotationClimberRightPublisher = myTable.getDoubleTopic("rotation_climber_right_rotations").publish();
        m_currentDrawClimberRightPublisher = myTable.getDoubleTopic("current_climber_right_amps").publish();

        m_neutralOut = new NeutralOut();
        m_followerRequest = new Follower(m_climberBreaker.getDeviceID(), true);
        m_retractedPositionVoltage = new PositionVoltage(0); //TODO: fix these
        m_retractedPositionVoltage.Position = 0; //TODO: fix these
        m_retractedPositionVoltage.EnableFOC = true;
    }

    @Override
    public void periodic() {
//        if (m_periodicIO.wantedState == State.STOWED) {
//            m_climberBreaker.setControl(m_neutralOut);
//        } else {
//            m_climberBreaker.setControl(m_retractedPositionVoltage);
//        }
//        m_climberBreaker.setControl(m_neutralOut);
        m_climberRio.setControl(m_followerRequest);
    }

    public void setVolts(final double volts) {
        m_climberBreaker.setControl(new VoltageOut(volts).withEnableFOC(true));
    }

    private static final double ARM_POSITION_STOP = -0.06;
    public void setVoltsProtected(final double voltsAttempt) {
        final double armPosition = Arm.getInstance().getShoulderRotation();

        if (armPosition < ARM_POSITION_STOP) {
            setVolts(0.0);
        } else {
            setVolts(voltsAttempt);
        }
    }

    public void setWantedState(final State state) {
        m_periodicIO.wantedState = state;
    }
}
