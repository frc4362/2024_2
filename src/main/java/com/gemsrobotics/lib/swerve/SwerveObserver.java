package com.gemsrobotics.lib.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.lib.allianceconstants.AllianceConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Objects;
import java.util.Optional;

public final class SwerveObserver implements Sendable {
    private final double m_maxSpeed;
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public SwerveObserver(final double maxSpeed) {
        m_maxSpeed = maxSpeed;
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Robot speeds for general checking */
//    private final FieldObject2d m_shadow = new FieldObject2d("Traj Robot");
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();
    private AllianceConstants m_allianceConstants = Constants.ALLIANCE_CONSTANTS_DEFAULT;
    private Optional<SwerveDriveState> latestState = Optional.empty();

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.2604, 0.2604);
    Translation2d m_frontRightLocation = new Translation2d(0.2604, -0.2604);
    Translation2d m_backLeftLocation = new Translation2d(-0.2604, 0.2604);
    Translation2d m_backRightLocation = new Translation2d(-0.2604, -0.2604);

    // Creating kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // Example module states
    private SwerveModuleState frontLeftState;
    private SwerveModuleState frontRightState;
    private SwerveModuleState backLeftState;
    private SwerveModuleState backRightState;
    private ChassisSpeeds chassisSpeeds;

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()
        });
        latestState = Optional.of(state);

        frontLeftState = state.ModuleStates[0];
        frontRightState = state.ModuleStates[1];
        backLeftState = state.ModuleStates[2];
        backRightState = state.ModuleStates[3];

        chassisSpeeds = m_kinematics.toChassisSpeeds(frontLeftState, frontRightState, backLeftState, backRightState);

        speed.set(Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
        velocityX.set(chassisSpeeds.vxMetersPerSecond);
        velocityY.set(chassisSpeeds.vyMetersPerSecond);
        odomPeriod.set(state.OdometryPeriod);

        /* Telemeterize the module's states */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * m_maxSpeed));

            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    public Translation2d getMeasuredVelocity() {
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    public void setShadowPose(final Pose2d pose) {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> latestState.map(s -> s.ModuleStates[0].angle.getRotations()).orElse(0.0), null);
        builder.addDoubleProperty("Front Left Velocity", () -> latestState.map(s -> s.ModuleStates[0].speedMetersPerSecond).orElse(0.0), null);

        builder.addDoubleProperty("Front Right Angle", () -> latestState.map(s -> s.ModuleStates[1].angle.getRotations()).orElse(0.0), null);
        builder.addDoubleProperty("Front Left Velocity", () -> latestState.map(s -> s.ModuleStates[1].speedMetersPerSecond).orElse(0.0), null);

        builder.addDoubleProperty("Back Left Angle", () -> latestState.map(s -> s.ModuleStates[2].angle.getRotations()).orElse(0.0), null);
        builder.addDoubleProperty("Front Left Velocity", () -> latestState.map(s -> s.ModuleStates[2].speedMetersPerSecond).orElse(0.0), null);

        builder.addDoubleProperty("Back Right Angle", () -> latestState.map(s -> s.ModuleStates[3].angle.getRotations()).orElse(0.0), null);
        builder.addDoubleProperty("Back Right Velocity", () -> latestState.map(s -> s.ModuleStates[3].speedMetersPerSecond).orElse(0.0), null);

        builder.addDoubleProperty("Robot Angle", () -> latestState.map(s -> s.Pose.getRotation().getRotations() + m_allianceConstants.getNorth().getRotations()).orElse(0.0), null);
    }

    public void setAllianceConstants(AllianceConstants allianceConstants) {
        m_allianceConstants = allianceConstants;
    }
}
