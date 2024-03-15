package com.gemsrobotics.frc2024.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.frc2024.ShotParam;
import com.gemsrobotics.lib.TalonUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public final class Arm implements Subsystem {
    private static final String NT_KEY = "arm";

    private static Arm INSTANCE;
    public static Arm getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new Arm();
        }

        return INSTANCE;
    }

    private final TalonFX m_elbow, m_shoulder;
    private final PeriodicIO m_periodicIO;
    private final PositionVoltage m_elbowRequest;// = new MotionMagicExpoVoltage(0); //TODO initilize with arm's starting position
    private final MotionMagicExpoVoltage m_shoulderRequest;// = new MotionMagicExpoVoltage(0); //TODO initilize with arm's starting position
    private final CoastOut m_coast = new CoastOut();


    private static final Rotation2d FLIP_ANGLE = Rotation2d.fromRotations(0.5);
    private static final double SHOULDER_MIN = -0.081; // going to assume this is the default position
    private static final double SHOULDER_MAX = 0.16925;
    private static final double ELBOW_MIN = 0;
    private static final double ELBOW_MAX = 0.33;
    private static final Rotation2d SHOULDER_DEFAULT = Rotation2d.fromRotations(SHOULDER_MIN);
    private static final Rotation2d SHOOTER_ANGLE_BOTTOM = Rotation2d.fromDegrees(0.0);

    private final StatusSignal<Double>
            m_rotationElbowSignal, m_rotationShoulderSignal,
            m_currentDrawElbow, m_currentDrawShoulder,
            m_motorVoltageElbow, m_motorVoltageShoulder,
            m_elbowErrorSignal, m_shoulderErrorSignal,
            m_referenceShoulderSignal, m_referenceElbowSignal;
    private final DoublePublisher
            m_rotationElbowPublisher, m_rotationShoulderPublisher,
            m_currentDrawElbowPublisher, m_currentDrawShoulderPublisher,
            m_voltsAppliedElbowPublisher, m_voltsAppliedShoulderPublisher,
            m_elbowReferencePublisher, m_shoulderReferencePublisher,
            m_fieldToElbowPublisher;
    public static class PeriodicIO {
        public State wantedState = State.STOWED;
        public double elbowRotation = 0.0;
        public double shoulderRotation = 0.0;
        public double currentDrawElbowAmps = 0.0;
        public double currentDrawShoulderAmps = 0.0;
    }

    public enum State {
        STOWED(SHOULDER_MIN, ELBOW_MIN),
//        AMP(0.099, 0.156),
        AMP(0.0765, 0.14),
//        AMP(0.126, 0.095),
        CLIMB_PLACE(1.65, 0.0),
//        CLIMB_PLACE_2(1.65, 0.34),
        TRAP(0.0, 0.0);

        public final double shoulderPositionsRotations, elbowPositionRotations;

        State(final double shoulder, final double elbow) {
            shoulderPositionsRotations = shoulder;
            elbowPositionRotations = elbow;
        }

        public double getExactShoulderValue() {
            return Rotation2d.fromRotations(shoulderPositionsRotations).getRotations();
        }

        public double getExactElbowValue() {
            return Rotation2d.fromRotations(elbowPositionRotations).getRotations();
        }
    }

    private Arm() {
        m_elbow = new TalonFX(19, Constants.AUX_BUS_NAME);
        m_shoulder = new TalonFX(18, Constants.AUX_BUS_NAME);

        m_periodicIO = new PeriodicIO();

        final var elbowConfig = new TalonFXConfiguration();
        elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        elbowConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        elbowConfig.MotionMagic.MotionMagicExpo_kV = 0.06;
        elbowConfig.MotionMagic.MotionMagicExpo_kA = 0.03;

//        elbowConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
//        elbowConfig.Feedback.FeedbackRemoteSensorID = 50;
        elbowConfig.Feedback.RotorToSensorRatio = 1;
        elbowConfig.Feedback.SensorToMechanismRatio = 163.64;//1;
        elbowConfig.Slot0.kS = 0.5;
        elbowConfig.Slot0.kV = 0.0;
        elbowConfig.Slot0.kA = 0.0;
        elbowConfig.Slot0.kP = 200.00;
        elbowConfig.Slot0.kI = 0.0;
        elbowConfig.Slot0.kD = 0.0;
        elbowConfig.Slot0.kG = 0.0;
        TalonUtils.configureTalon(elbowConfig, m_elbow);
        m_elbow.setPosition(ELBOW_MIN);

        final var shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoulderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shoulderConfig.MotionMagic.MotionMagicExpo_kV = 0.5;
        shoulderConfig.MotionMagic.MotionMagicExpo_kA = 1.0;
//        shoulderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
//        shoulderConfig.Feedback.FeedbackRemoteSensorID = 51;
        shoulderConfig.Feedback.RotorToSensorRatio = 1; // used  to be 157.5 : 1
        shoulderConfig.Feedback.SensorToMechanismRatio = 112.5;
        shoulderConfig.Slot0.kS = 0.5;
        shoulderConfig.Slot0.kV = 0.0;
        shoulderConfig.Slot0.kA = 0.0;
        shoulderConfig.Slot0.kP = 100.0;
        shoulderConfig.Slot0.kI = 0.0;
        shoulderConfig.Slot0.kD = 0.0;
        shoulderConfig.Slot0.kG = 0.0;
        TalonUtils.configureTalon(shoulderConfig, m_shoulder);
        m_shoulder.setPosition(SHOULDER_MIN);

        // grab status signals to monitor velocity (RPS) and torque current (A)
        m_rotationElbowSignal = m_elbow.getPosition();
        m_rotationShoulderSignal = m_shoulder.getPosition();

        m_currentDrawElbow = m_elbow.getTorqueCurrent();
        m_currentDrawShoulder = m_shoulder.getTorqueCurrent();

        m_motorVoltageElbow = m_elbow.getMotorVoltage();
        m_motorVoltageShoulder = m_shoulder.getMotorVoltage();

        m_referenceElbowSignal = m_elbow.getClosedLoopReference();
        m_referenceShoulderSignal = m_shoulder.getClosedLoopReference();

        m_elbowErrorSignal = m_elbow.getClosedLoopError();
        m_shoulderErrorSignal = m_shoulder.getClosedLoopError();

        m_shoulderRequest = new MotionMagicExpoVoltage(SHOULDER_MIN);
        m_shoulderRequest.EnableFOC = true;
        m_elbowRequest = new PositionVoltage(ELBOW_MIN);
        m_elbowRequest.EnableFOC = true;

        // configure logging of velocity (RPS) and torque current (A)
        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
        m_rotationElbowPublisher = myTable.getDoubleTopic("elbow_rotations").publish();
        m_elbowReferencePublisher = myTable.getDoubleTopic("elbow_reference").publish();
        m_shoulderReferencePublisher = myTable.getDoubleTopic("shoulder_reference").publish();
        m_rotationShoulderPublisher = myTable.getDoubleTopic("shoulder_rotations").publish();
        m_currentDrawElbowPublisher = myTable.getDoubleTopic("current_elbow_amps").publish();
        m_currentDrawShoulderPublisher = myTable.getDoubleTopic("current_shoulder_amps").publish();
        m_voltsAppliedElbowPublisher = myTable.getDoubleTopic("volts_applied_elbow").publish();
        m_voltsAppliedShoulderPublisher = myTable.getDoubleTopic("volts_applied_shoulder").publish();
        m_fieldToElbowPublisher = myTable.getDoubleTopic("elbow_to_floor_degrees").publish();
    }

    @Override
    public void periodic() {
        m_periodicIO.shoulderRotation = m_rotationShoulderSignal.refresh().getValue();
        m_rotationShoulderPublisher.set(m_periodicIO.shoulderRotation);
        m_periodicIO.currentDrawShoulderAmps = m_currentDrawShoulder.refresh().getValue();
        m_currentDrawShoulderPublisher.set(m_periodicIO.currentDrawShoulderAmps);
        m_voltsAppliedShoulderPublisher.set(m_motorVoltageShoulder.refresh().getValue());
        m_shoulderReferencePublisher.set(m_referenceShoulderSignal.refresh().getValue());

        m_periodicIO.elbowRotation = m_rotationElbowSignal.refresh().getValue();
        m_rotationElbowPublisher.set(m_periodicIO.elbowRotation);
        m_periodicIO.currentDrawElbowAmps = m_currentDrawElbow.refresh().getValue();
        m_currentDrawElbowPublisher.set(m_periodicIO.currentDrawElbowAmps);
        m_voltsAppliedElbowPublisher.set(m_motorVoltageElbow.refresh().getValue());
        m_elbowReferencePublisher.set(m_referenceElbowSignal.refresh().getValue());

        m_fieldToElbowPublisher.set(calculateFieldToElbow().getDegrees());
    }

    public void setOff() {
        m_elbow.setControl(m_coast);
        m_shoulder.setControl(m_coast);
    }

    public void setFinalClimb() {
        m_shoulder.setControl(new CoastOut());
        m_elbow.setControl(m_elbowRequest.withPosition(0.0));
    }

    public void setAngles(final Rotation2d shoulderAngle, final Rotation2d elbowAngle) {
        //sets the goal angles for both the shoulder and the elbow, with max/min angle protection
        m_elbowRequest.FeedForward = 0.0 * calculateFieldToElbow().getCos(); // TODO: tune kG constants

        double shoulderAttempt = shoulderAngle.getRotations();
        shoulderAttempt = Math.max(SHOULDER_MIN, Math.min(SHOULDER_MAX, shoulderAttempt));
        m_shoulderRequest.Position = shoulderAttempt;
        m_shoulder.setControl(m_shoulderRequest);

        double elbowAttempt = elbowAngle.getRotations();
        elbowAttempt = Math.max(ELBOW_MIN, Math.min(ELBOW_MAX, elbowAttempt));
        m_elbowRequest.Position = elbowAttempt;
        m_elbow.setControl(m_elbowRequest);
    }

    public void setShootingAngle(ShotParam parameters) {
        // as of right now we are assuming that we are NOT climbing, therfore we should just delegate the entire angle to elbow
        setAngles(SHOULDER_DEFAULT, parameters.getAngle().minus(SHOOTER_ANGLE_BOTTOM));
    }

    private static final Rotation2d COG_OFFSET = Rotation2d.fromDegrees(1.4);
    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(m_periodicIO.elbowRotation).plus(SHOOTER_ANGLE_BOTTOM);
    }

    public double getShoulderRotation() {
        return m_periodicIO.shoulderRotation;
    }

    private Rotation2d calculateFieldToElbow() {
        // rho_prime = elbow rotations
        // theta = shoulder rotations

        // rho_prime (degrees) = -(rho - 180)
        // elbow angle to shoulder zero = rho_prime + theta
        Rotation2d rho_prime = Rotation2d.fromRotations(m_periodicIO.elbowRotation);
        rho_prime = rho_prime.plus(Rotation2d.fromRotations(m_periodicIO.shoulderRotation));
        return rho_prime.unaryMinus().minus(COG_OFFSET);
    }

    public void setWantedState(final State state) {
        m_periodicIO.wantedState = state;
        setAngles(Rotation2d.fromRotations(state.shoulderPositionsRotations), Rotation2d.fromRotations(state.elbowPositionRotations));
    }

    public boolean atReference(final double toleranceShoulder, final double toleranceElbow) {
        return MathUtil.isNear(m_elbowErrorSignal.refresh().getValue(), 0.0, toleranceElbow)
               && MathUtil.isNear(m_shoulderErrorSignal.refresh().getValue(), 0.0, toleranceShoulder);
    }

    public boolean atReference(final ShotParam param) {
        return MathUtil.isNear(m_periodicIO.elbowRotation, param.getAngle().minus(SHOOTER_ANGLE_BOTTOM).getRotations(), 0.005);
    }

    public boolean atReference() {
        return atReference(0.01, 0.01);
    }
}