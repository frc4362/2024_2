package com.gemsrobotics.frc2024.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.lib.TalonUtils;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public class Bender implements Subsystem {

    private static final String NT_KEY = "bender";

    private static Bender INSTANCE;
    public static Bender getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new Bender();
        }
        return INSTANCE;
    }

    private final TalonFX m_bender;
    private final PeriodicIO m_periodicIO;

    private final StatusSignal<Double> m_rotationBenderSignal, m_currentDrawBender;
    private final DoublePublisher m_rotationBenderPublisher, m_currentDrawBenderPublisher;

    public static class PeriodicIO {
        public State wantedState = State.STOWED;
        public double rotationBenderRotations = 0.0;
        public double currentDrawBenderAmps = 0.0;
    }

    public enum State {
        STOWED(0.0),
        DEPLOYED(16.00),
        WIGGLING(16.50); // used to be 16.5

        public double rotations;

        State(final double r) {
            rotations = r;
        }
    }

    private NeutralOut m_neutralOut;
    private PositionVoltage m_deployedPositionVoltage;
    private boolean m_doSlam;

    private Bender() {
        m_bender = new TalonFX(26, Constants.AUX_BUS_NAME);

        m_periodicIO = new PeriodicIO();
        m_doSlam = false;

        final var benderConfig = new TalonFXConfiguration();
        benderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        benderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        benderConfig.Feedback.RotorToSensorRatio = 1.0;
        benderConfig.Feedback.SensorToMechanismRatio = 1.0;//40.0;
        // velocity gains
        benderConfig.Slot0.kP = 24.0 / 16.0; // make it fast LOL
        benderConfig.Slot0.kI = 0.0;
        benderConfig.Slot0.kD = 0.0;
        TalonUtils.configureTalon(benderConfig, m_bender);

        // grab status signals to monitor velocity (RPS) and torque current (A)
        m_rotationBenderSignal = m_bender.getPosition();
        m_currentDrawBender = m_bender.getTorqueCurrent();

        // configure logging of velocity (RPS) and torque current (A)
        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
        m_rotationBenderPublisher = myTable.getDoubleTopic("rotation_bender_rotations").publish();
        m_currentDrawBenderPublisher = myTable.getDoubleTopic("current_bender_amps").publish();

        m_neutralOut = new NeutralOut();
        m_deployedPositionVoltage = new PositionVoltage(0);
        m_deployedPositionVoltage.Position = 0;
        m_deployedPositionVoltage.EnableFOC = true;
    }

    @Override
    public void periodic() {
        m_periodicIO.rotationBenderRotations = m_rotationBenderSignal.refresh().getValue();
        m_rotationBenderPublisher.set(m_periodicIO.rotationBenderRotations);
        m_periodicIO.currentDrawBenderAmps = m_currentDrawBender.refresh().getValue();
        m_currentDrawBenderPublisher.set(m_periodicIO.currentDrawBenderAmps);

//        m_bender.setControl(new NeutralOut());

        if (m_doSlam) {
            m_bender.setControl(m_deployedPositionVoltage.withPosition(State.DEPLOYED.rotations));
        }else if (m_periodicIO.wantedState == State.WIGGLING) {
            m_bender.setControl(m_deployedPositionVoltage.withPosition(m_periodicIO.wantedState.rotations));// + 3 * Math.sin(3 * Timer.getFPGATimestamp())));
        } else {
            m_bender.setControl(m_deployedPositionVoltage.withPosition(m_periodicIO.wantedState.rotations));
        }
    }

    public void setDoSlam(final boolean doSlammy) {
        m_doSlam = doSlammy;
    }

    public void setWantedState(final State state) {
        m_periodicIO.wantedState = state;
    }
}
