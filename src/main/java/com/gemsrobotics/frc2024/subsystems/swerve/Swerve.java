package com.gemsrobotics.frc2024.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.*;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.frc2024.subsystems.NoteDetector;
import com.gemsrobotics.frc2024.subsystems.ShotType;
import com.gemsrobotics.lib.allianceconstants.AllianceConstants;
import com.gemsrobotics.lib.math.Rotation2dPlus;
import com.gemsrobotics.lib.math.Units;
import com.gemsrobotics.lib.swerve.FieldCentricEvasion;
import com.gemsrobotics.lib.swerve.SwerveObserver;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static java.lang.Math.abs;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public final class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final String NT_KEY = "swerve";

    // sim fields
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    public static final double ROTATION_RATE_SCALAR = 0.7;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // singleton fields
    private static Swerve INSTANCE = null;

    public static Swerve getInstance() {
        if (Objects.isNull(INSTANCE)) {
            final SwerveConstants constants = new SwerveConstantsProd();
            INSTANCE = new Swerve(constants);
        }

        return INSTANCE;
    }

    private static final Translation2d FAR_AIM_ADJUSTMENT = new Translation2d(0.0, Units.inches2Meters(-6.0));

    // open loop constants
    private static final Rotation2d FLIP_ANGLE = Rotation2d.fromDegrees(180);
    private static final double DEADBAND = 0.025;

    // provide some complex measurements to outside classes
    private final SwerveObserver m_telemetry;
    private final SwerveConstants m_constants;
    private final SwerveRequest.Idle m_idleRequest;
    private final SwerveRequest.ApplyChassisSpeeds m_autoRequest;
    private final FieldCentricEvasion m_teleopRequest;
    private final SwerveRequest.FieldCentricFacingAngle m_aimingRequest;
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest;
    private final ChoreoControlFunction m_locationFeedbackController;
    private final List<DoublePublisher> m_voltagePublishers;
    private final List<DoublePublisher> m_torquePublishers;
    private final DoublePublisher m_trajectoryErrorPublisher;
    private final DoublePublisher m_velocityErrorPublisher;
    private final DoublePublisher m_angleToGoal;
    private final List<StatusSignal<Double>> m_voltageSignals, m_torqueSignals;
    private ShotType m_shotType;
    private BaseStatusSignal[] m_baseSignals;
    private AllianceConstants m_allianceConstants;
	private Optional<Rotation2d> m_maintainHeadingGoal;

    private SwerveRequest.SysIdSwerveTranslation driveVoltageRequest = new SwerveRequest.SysIdSwerveTranslation();

    private SysIdRoutine m_driveSysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
                    new SysIdRoutine.Mechanism(
                            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVolts(volts)),
                            null,
                            this));

    private Swerve(final SwerveConstants constants) {
        super(constants.getIds(), constants.getFrontLeft(), constants.getFrontRight(), constants.getBackLeft(), constants.getBackRight());
        m_constants = constants;

        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_telemetry = new SwerveObserver(constants.getMaxSpeedMetersPerSecond());
        registerTelemetry(m_telemetry::telemeterize);

        m_idleRequest = new SwerveRequest.Idle();

        // config tracking request
        m_autoRequest = new SwerveRequest.ApplyChassisSpeeds();
        m_autoRequest.DriveRequestType = SwerveModule.DriveRequestType.Velocity;
        m_autoRequest.SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        // config teleop request
        m_teleopRequest = new FieldCentricEvasion();
        m_teleopRequest.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        m_teleopRequest.SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        // config driving while aiming request
        m_aimingRequest = new SwerveRequest.FieldCentricFacingAngle();
        m_aimingRequest.HeadingController = new PhoenixPIDController(12, 0.0, 0.7);
        m_aimingRequest.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        m_aimingRequest.SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;
        m_aimingRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        m_aimingRequest.HeadingController.setTolerance(Units.degrees2Rads(0.0));

        // brake request
        m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
        m_brakeRequest.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        m_brakeRequest.SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        m_locationFeedbackController = Choreo.choreoSwerveController(
                new PIDController(2.5, 0.0, 0.0),
                new PIDController(2.5, 0.0, 0.0),
                new PIDController(2.5, 0.0, 0.0)
        );

        m_pigeon2.setYaw(0); // reset gyro

        // configure logging
        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);

        m_voltagePublishers = new ArrayList<>(4);
        m_voltagePublishers.add(myTable.getDoubleTopic("front_left_voltage").publish());
        m_voltagePublishers.add(myTable.getDoubleTopic("front_right_voltage").publish());
        m_voltagePublishers.add(myTable.getDoubleTopic("back_left_voltage").publish());
        m_voltagePublishers.add(myTable.getDoubleTopic("back_right_voltage").publish());

        m_torquePublishers = new ArrayList<>(4);
        m_torquePublishers.add(myTable.getDoubleTopic("front_left_torque").publish());
        m_torquePublishers.add(myTable.getDoubleTopic("front_right_torque").publish());
        m_torquePublishers.add(myTable.getDoubleTopic("back_left_torque").publish());
        m_torquePublishers.add(myTable.getDoubleTopic("back_right_torque").publish());

        m_trajectoryErrorPublisher = myTable.getDoubleTopic("trajectory_error").publish();
        m_trajectoryErrorPublisher.setDefault(0.0);

        m_velocityErrorPublisher = myTable.getDoubleTopic("velocity_error").publish();
        m_velocityErrorPublisher.setDefault(0.0);

        m_voltageSignals = new ArrayList<>(4);
        m_torqueSignals = new ArrayList<>(4);

        for (final var module : Modules) {
            var closedLoopRampConfigs = new ClosedLoopRampsConfigs();
            var openLoopRampConfigs = new OpenLoopRampsConfigs();
            var driveTalonFXConfigurator = module.getDriveMotor().getConfigurator();
            closedLoopRampConfigs.DutyCycleClosedLoopRampPeriod = 0;
            closedLoopRampConfigs.VoltageClosedLoopRampPeriod = 0;
            openLoopRampConfigs.VoltageOpenLoopRampPeriod = 0.02;
            driveTalonFXConfigurator.apply(closedLoopRampConfigs);
            driveTalonFXConfigurator.apply(openLoopRampConfigs);

            var angleCurrentLimits = new CurrentLimitsConfigs();
            var angleTalonFXConfigurator = module.getSteerMotor().getConfigurator();
            angleCurrentLimits.SupplyCurrentLimitEnable = true;
            angleCurrentLimits.SupplyCurrentLimit = 40;
            angleCurrentLimits.SupplyCurrentThreshold = 40;
            angleCurrentLimits.SupplyTimeThreshold = 1.0;
            angleCurrentLimits.StatorCurrentLimitEnable = false;
            angleCurrentLimits.StatorCurrentLimit = 60;
            angleTalonFXConfigurator.apply(angleCurrentLimits);

            var driveCurrentLimits = new CurrentLimitsConfigs();
            driveCurrentLimits.SupplyCurrentLimitEnable = true;
            driveCurrentLimits.SupplyCurrentThreshold = 120;
            driveCurrentLimits.StatorCurrentLimitEnable = true;
            driveCurrentLimits.StatorCurrentLimit = 92.5;
            driveTalonFXConfigurator.apply(driveCurrentLimits);

            m_voltageSignals.add(module.getDriveMotor().getMotorVoltage());
            m_torqueSignals.add(module.getDriveMotor().getTorqueCurrent());
        }

        configNeutralMode(NeutralModeValue.Brake);

        m_angleToGoal = myTable.getDoubleTopic("shooter_to_goal_degrees").publish();

        m_baseSignals = new BaseStatusSignal[] {
                m_voltageSignals.get(0),
                m_voltageSignals.get(1),
                m_voltageSignals.get(2),
                m_voltageSignals.get(3),
                m_torqueSignals.get(0),
                m_torqueSignals.get(1),
                m_torqueSignals.get(2),
                m_torqueSignals.get(3)
        };

        m_shotType = ShotType.SPEAKER;
        m_allianceConstants = Constants.ALLIANCE_CONSTANTS_DEFAULT;
		m_maintainHeadingGoal = Optional.empty();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_baseSignals);

        m_angleToGoal.set(getAimingError().getDegrees());
        for (int i = 0; i < m_voltagePublishers.size(); i++) {
            m_voltagePublishers.get(i).set(m_voltageSignals.get(i).getValue());
        }

        for (int i = 0; i < m_torquePublishers.size(); i++) {
            m_torquePublishers.get(i).set(m_torqueSignals.get(i).getValue());
        }
    }

    public void setShotType(final ShotType shotType) {
        m_shotType = shotType;
    }

    public Rotation2d getAimingError() {
        return getAngleToGoal().minus(getState().Pose.getRotation());
    }

    /**
     * Creates the field-relative open-loop translation from an alliance-relative vector.
     * ie. x and y are translated based on the current alliance of the drive train, so that 1.0 is still towards
     * the other end of the field.
     * @param fieldTranslationVector A vector in alliance-relative space. 1.0 is towards the other alliance wall.
     * @return A vector in field-space, corrected for ease of use. Flipped for red alliance
     */
    private Translation2d makeOpenLoopTranslation(final Translation2d fieldTranslationVector, final boolean doAxisLock) {
        double magnitude = MathUtil.applyDeadband(fieldTranslationVector.getNorm(), DEADBAND, 1.0);
        // scale for low-range movements
        magnitude = Math.pow(magnitude, 1.5);

        var direction = new Rotation2dPlus(fieldTranslationVector.getAngle());
        if (doAxisLock) {
            final var nearestPole = direction.getNearestPole();
            if (abs(direction.minus(nearestPole).getDegrees()) < 5) {
                direction = nearestPole;
            }
        }

        // flip movement based on alliance
        final var allianceDirection = direction.rotateBy(m_allianceConstants.getNorth());
        return new Translation2d(magnitude * m_constants.getMaxSpeedMetersPerSecond(), allianceDirection);
    }

    /**
     * Translates and rotates the chassis according to joysticks.
     * @param joystickTranslation The joystick input to drive with
     * @param joystickRotationRate The joystick input to rotate with
     * @param evading Should we be evading? (Rotating the chassis around one wheel as we translate)
     */
    public void setOpenLoopJoysticks(
            final Translation2d joystickTranslation,
            final double joystickRotationRate,
            final boolean lockCardinals,
            final boolean evading
    ) {
        final var dbRotation = MathUtil.applyDeadband(joystickRotationRate, DEADBAND, 1.0)
                                       * m_constants.getMaxAngularRateRadiansPerSecond()
                                       * ROTATION_RATE_SCALAR;
		final var scaledTranslation = makeOpenLoopTranslation(joystickTranslation, lockCardinals);

        // if we're basically just sitting still
        if (dbRotation == 0.0 && scaledTranslation.getNorm() < 0.01) {
            m_maintainHeadingGoal = Optional.empty();
            setControl(m_idleRequest);
            return;
        }

        // if we are not commanding a turn, keep your heading as you translate
        if (dbRotation == 0.0) {
			if (m_maintainHeadingGoal.isEmpty()) {
				m_maintainHeadingGoal = Optional.of(getState().Pose.getRotation());
			}

			setOpenLoopFaceHeading(scaledTranslation, m_maintainHeadingGoal.get());
			return;
        }

        // otherwise, scale your turning and clear your maintain heading goal
        final var scaledRotation = dbRotation;//Math.copySign(Math.pow(Math.abs(dbRotation), 1.75), dbRotation);
		m_maintainHeadingGoal = Optional.empty();

        m_teleopRequest.VelocityX = scaledTranslation.getX();
        m_teleopRequest.VelocityY = scaledTranslation.getY();
        m_teleopRequest.Evading = evading;
        m_teleopRequest.RotationalRate = scaledRotation;

        setControl(m_teleopRequest);
    }

    private static final double kP = 4.0;
    public void setOpenLoopNoteChasing(final Translation2d joystickTranslation, final double joystickRotationRate) {
        Optional<Rotation2d> vehicleToNoteRotation = NoteDetector.getInstance().getVehicleToNoteRotation();

        // if we don't see a note, exit early
        if (vehicleToNoteRotation.isEmpty()) {
            setOpenLoopJoysticks(joystickTranslation, joystickRotationRate, false, false);
            return;
        }

        final Rotation2d fieldToVehicleRotation = getState().Pose.getRotation();
        final Rotation2d vehicleToNote = vehicleToNoteRotation.get();
        final var correction = kP * vehicleToNote.getRadians();
        final var scaledTranslation = makeOpenLoopTranslation(joystickTranslation, false);

        // difference between the way the robot is facing and the way the robot is moving
        final var movementDirectionRobotRelative = scaledTranslation.getAngle().minus(getState().Pose.getRotation());
        // if you're moving within the 90 degrees facing the note
        if (abs(movementDirectionRobotRelative.minus(vehicleToNote).getDegrees()) < 45.0) {
            final var pointedTranslation = new Translation2d(scaledTranslation.getNorm(), fieldToVehicleRotation.plus(vehicleToNote));

            m_maintainHeadingGoal = Optional.empty();

            m_teleopRequest.VelocityX = pointedTranslation.getX();
            m_teleopRequest.VelocityY = pointedTranslation.getY();
            m_teleopRequest.Evading = false;
            m_teleopRequest.RotationalRate = correction;

            setControl(m_teleopRequest);
        // otherwise be normal
        } else {
            setOpenLoopJoysticks(joystickTranslation, joystickRotationRate, false, false);
        }
    }

    public boolean approachNote() {
        Optional<Rotation2d> vehicleToNoteRotation = NoteDetector.getInstance().getVehicleToNoteRotation();

        // if we don't see a note, exit early
        if (vehicleToNoteRotation.isEmpty()) {
            return false;
        }

        final Rotation2d fieldToVehicleRotation = getState().Pose.getRotation();
        final Rotation2d vehicleToNote = vehicleToNoteRotation.get();
        final var correction = kP * vehicleToNote.getRadians();
        final var pointedTranslation = new Translation2d(1, fieldToVehicleRotation.plus(vehicleToNote));

        m_maintainHeadingGoal = Optional.empty();

        m_teleopRequest.VelocityX = pointedTranslation.getX();
        m_teleopRequest.VelocityY = pointedTranslation.getY();
        m_teleopRequest.Evading = false;
        m_teleopRequest.RotationalRate = correction;

        setControl(m_teleopRequest);
        return true;
    }

	public void setOpenLoopFaceHeadingJoysticks(final Translation2d joystickTranslation, final Rotation2d heading) {
		final var scaledTranslation = makeOpenLoopTranslation(joystickTranslation, true);
		setOpenLoopFaceHeading(scaledTranslation, heading);
	}

    /**
     * Translates the chassis while attempting to face a certain angle heading.
     * @param openLoopTranslation The translation input to drive with
     * @param heading The heading which we are attempting to maintain.
     */
    private void setOpenLoopFaceHeading(final Translation2d openLoopTranslation, final Rotation2d heading) {
        m_aimingRequest.VelocityX = openLoopTranslation.getX();
        m_aimingRequest.VelocityY = openLoopTranslation.getY();
        m_aimingRequest.TargetDirection = heading;

        setControl(m_aimingRequest);
    }

    /**
     * Translates the chassis while attempting to follow a point on the field
     * @param joystickTranslation The joystick input to drive with.
     * @param goal The point which we are attempting to face.
     */
    public void setOpenLoopFaceLocation(final Translation2d joystickTranslation, final Translation2d goal) {
        final var directionToLocation = getState().Pose.getTranslation()
                .minus(goal)
                .getAngle();
//                .rotateBy(FLIP_ANGLE);
        setOpenLoopFaceHeadingJoysticks(joystickTranslation, directionToLocation);
    }

    /**
     * Automatically aims at the target which has been configured.
     * @param joystickTranslation The joystick input to drive with.
     */
    public void setAimingAtGoal(final Translation2d joystickTranslation) {
        setOpenLoopFaceHeadingJoysticks(joystickTranslation, getAngleToGoal());
    }

    /**
     * Configures the locations on the field with which we are concerned
     * @param constants The locations for the relevant alliance features, such as North and the Speaker
     */
    public void setAllianceConstants(final AllianceConstants constants) {
        m_allianceConstants = constants;
        m_telemetry.setAllianceConstants(m_allianceConstants);
    }

    /**
     * Sets the wheels to X-formation
     */
    public void setBraking() {
        setControl(m_brakeRequest);
    }

    private Translation2d getSpeakerLocation() {
        var speakerLocation = m_allianceConstants.getSpeakerLocationMeters();
        if (getState().Pose.getTranslation().getDistance(speakerLocation) > 5.0) {
            speakerLocation = speakerLocation.plus(FAR_AIM_ADJUSTMENT);
        }

        return speakerLocation;
    }

    private Translation2d getGoalLocation() {
        return switch (m_shotType) {
            case SPEAKER -> getSpeakerLocation();
            case AMP_FEED -> m_allianceConstants.getAmpZoneLocationMeters();
            case MID_FEED -> m_allianceConstants.getMiddleFeedLocationMeters();
        };
    }

    private Rotation2d getAngleToGoal() {
        return getState().Pose.getTranslation().minus(getGoalLocation()).getAngle();
    }

    private boolean shouldFlipState() {
        return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false);
    }

    public Command resetOdometryOnTrajectory(final String trajectoryName) {
        final var trajectory = Choreo.getTrajectory(trajectoryName);
        return new InstantCommand(() -> {
            final var initialState = trajectory.getInitialState();
            final var maybeFlippedState = shouldFlipState() ? initialState.flipped() : initialState;
            seedFieldRelative(maybeFlippedState.getPose());
        });
    }

    public Command getTrackTrajectoryCommand(final String trajectoryName, final boolean resetPose) {
        final var trajectory = Choreo.getTrajectory(trajectoryName);
        final var trackCommand = Choreo.choreoSwerveCommand(
                trajectory,
                () -> getState().Pose,
                (Pose2d currentPose, ChoreoTrajectoryState referencePose) -> {
                    m_trajectoryErrorPublisher.set(currentPose.getTranslation().getDistance(referencePose.getPose().getTranslation()));
                    m_velocityErrorPublisher.set(Math.hypot(referencePose.velocityX, referencePose.velocityY) - m_telemetry.getMeasuredVelocity().getNorm());
                    return m_locationFeedbackController.apply(currentPose, referencePose);
                },
                speeds -> setControl(m_autoRequest.withSpeeds(speeds)),
                this::shouldFlipState
        );

        return Commands.sequence(
                new InstantCommand(() -> {
                    final var initialState = trajectory.getInitialState();
                    final var maybeFlippedState = shouldFlipState() ? initialState.flipped() : initialState;
                    seedFieldRelative(maybeFlippedState.getPose());
                }).onlyIf(() -> resetPose),
                trackCommand,
                new InstantCommand(() -> setControl(m_idleRequest))
        ).withName("TrackCommand[\"" + trajectoryName + "\"]");
    }

    public SwerveObserver getObserver() {
        return m_telemetry;
    }

    public Command runDriveQuasiTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }
}
