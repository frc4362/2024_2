package com.gemsrobotics.frc2024.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.*;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.gemsrobotics.frc2024.Constants;
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
import edu.wpi.first.networktables.DoubleSubscriber;
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

    // open loop constants
    private static final Rotation2d FLIP_ANGLE = Rotation2d.fromDegrees(180);
    private static final double DEADBAND = 0.15;

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
    private final DoublePublisher m_trajectoryErrorPublisher;
    private final DoublePublisher m_velocityErrorPublisher;
    private final DoublePublisher m_angleToSpeaker;
    private final List<StatusSignal<Double>> m_voltageSignals;
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
        m_aimingRequest.HeadingController = new PhoenixPIDController(12, 0.0, 0.35);
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

        m_trajectoryErrorPublisher = myTable.getDoubleTopic("trajectory_error").publish();
        m_trajectoryErrorPublisher.setDefault(0.0);

        m_velocityErrorPublisher = myTable.getDoubleTopic("velocity_error").publish();
        m_velocityErrorPublisher.setDefault(0.0);

        m_voltageSignals = new ArrayList<>(4);

        for (final var module : Modules) {
            var closedLoopRampConfigs = new ClosedLoopRampsConfigs();
            var driveTalonFXConfigurator = module.getDriveMotor().getConfigurator();
            closedLoopRampConfigs.DutyCycleClosedLoopRampPeriod = 0;
            closedLoopRampConfigs.VoltageClosedLoopRampPeriod = 0;
            driveTalonFXConfigurator.apply(closedLoopRampConfigs);

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
            driveCurrentLimits.StatorCurrentLimit = 75;
            driveTalonFXConfigurator.apply(driveCurrentLimits);

            m_voltageSignals.add(module.getDriveMotor().getMotorVoltage());
        }

        m_angleToSpeaker = myTable.getDoubleTopic("shooter_to_speaker_degrees").publish();

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
        m_angleToSpeaker.set(getAimingError().getDegrees());
        for (int i = 0; i < m_voltagePublishers.size(); i++) {
            m_voltagePublishers.get(i).set(m_voltageSignals.get(i).refresh().getValue());
        }
    }

    public Rotation2d getAimingError() {
        return getAngleToSpeaker().minus(getState().Pose.getRotation());
    }

    /**
     * Creates the field-relative open-loop translation from an alliance-relative vector.
     * ie. x and y are translated based on the current alliance of the drive train, so that 1.0 is still towards
     * the other end of the field.
     * @param fieldTranslationVector A vector in alliance-relative space. 1.0 is towards the other alliance wall.
     * @return A vector in field-space, corrected for ease of use. Flipped for red alliance
     */
    private Translation2d makeOpenLoopTranslation(final Translation2d fieldTranslationVector) {
        double magnitude = MathUtil.applyDeadband(fieldTranslationVector.getNorm(), DEADBAND, 1.0);
        // scale for low-range movements
        magnitude = Math.pow(magnitude, 1.5);

        var direction = new Rotation2dPlus(fieldTranslationVector.getAngle());
        final var nearestPole = direction.getNearestPole();
        if (Math.abs(direction.minus(nearestPole).getDegrees()) < 5) {
            direction = nearestPole;
        }

        // flip movement based on alliance
        final var allianceDirection = direction.rotateBy(m_allianceConstants.getNorth());
        return new Translation2d(magnitude * m_constants.getMaxSpeedMetersPerSecond(), allianceDirection);
    }

    public void setOpenLoopAiming(
            final Translation2d joystickTranslation,
            final double aimingFeedback
    ) {
        final var scaledTranslation = makeOpenLoopTranslation(joystickTranslation);

        m_teleopRequest.VelocityX = scaledTranslation.getX();
        m_teleopRequest.VelocityY = scaledTranslation.getY();
        m_teleopRequest.Evading = false;
        m_teleopRequest.RotationalRate = aimingFeedback * m_constants.getMaxAngularRateRadiansPerSecond() * 0.5;

        setControl(m_teleopRequest);
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
            final boolean evading
    ) {
        final var dbRotation = MathUtil.applyDeadband(joystickRotationRate, DEADBAND, 1.0)
                                       * m_constants.getMaxAngularRateRadiansPerSecond()
                                       * ROTATION_RATE_SCALAR;
		final var scaledTranslation = makeOpenLoopTranslation(joystickTranslation);

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

	public void setOpenLoopFaceHeadingJoysticks(final Translation2d joystickTranslation, final Rotation2d heading) {
		final var scaledTranslation = makeOpenLoopTranslation(joystickTranslation);
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
     * Automatically aims at the speaker which has been configured.
     * @param joystickTranslation The joystick input to drive with.
     */
    public void setAimingAtGoal(final Translation2d joystickTranslation) {
        setOpenLoopFaceHeadingJoysticks(joystickTranslation, getAngleToSpeaker());
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

    private Rotation2d getAngleToSpeaker() {
        return getState().Pose.getTranslation().minus(m_allianceConstants.getSpeakerLocationMeters()).getAngle();
    }

    private boolean shouldFlipState() {
        return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false);
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
        );
    }

    public SwerveObserver getObserver() {
        return m_telemetry;
    }

    public Command runDriveQuasiTest(SysIdRoutine.Direction direction)
    {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }
}
