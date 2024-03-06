package com.gemsrobotics.frc2024.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstantsTesting implements SwerveConstants {
    public static final double MaxSpeed = 5.2; // 6 meters per second desired top speed
    /** Analytically derived, Radians per Second */
    public static final double MaxAngularRate = 7.061561863203472;//(MaxSpeed / Math.hypot(SwerveConstants.ChassisWidthMeters / 2.0, SwerveConstants.ChassisWidthMeters / 2.0)) / 2.0;

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.05)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    private static final double kSpeedAt12VoltsMps = 5.2;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 2;

    private static final double kDriveGearRatio = 5.5;
    private static final double kSteerGearRatio = 10.285714285714286;
    private static final double kWheelRadiusInches = 2.1;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "swerve";
    private static final int kPigeonId = 0;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 5;
//     private static final int kFrontLeftDriveMotorId = 5;
//     private static final int kFrontLeftSteerMotorId = 3;
    private static final int kFrontLeftEncoderId = 9;
    private static final double kFrontLeftEncoderOffset = -0.195556640625;

    private static final double kFrontLeftXPosInches = 10.25;
    private static final double kFrontLeftYPosInches = 10.25;

    // Front Right
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 8;
//     private static final int kFrontRightDriveMotorId = 8;
//     private static final int kFrontRightSteerMotorId = 4;
    private static final int kFrontRightEncoderId = 12;
    private static final double kFrontRightEncoderOffset = -0.398681640625;

    private static final double kFrontRightXPosInches = 10.25;
    private static final double kFrontRightYPosInches = -10.25;

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 7;
//     private static final int kBackLeftDriveMotorId = 7;
//     private static final int kBackLeftSteerMotorId = 1;
    private static final int kBackLeftEncoderId = 11;
    private static final double kBackLeftEncoderOffset = 0.0986328125;

    private static final double kBackLeftXPosInches = -10.25;
    private static final double kBackLeftYPosInches = 10.25;

    // Back Right
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 6;
//     private static final int kBackRightDriveMotorId = 6;
//     private static final int kBackRightSteerMotorId = 2;
    private static final int kBackRightEncoderId = 10;
    private static final double kBackRightEncoderOffset = 0.235595703125;

    private static final double kBackRightXPosInches = -10.25;
    private static final double kBackRightYPosInches = -10.25;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(kFrontLeftXPosInches, kFrontLeftYPosInches),
        new Translation2d(kFrontRightXPosInches, kFrontRightYPosInches),
        new Translation2d(kBackRightXPosInches, kBackRightYPosInches),
        new Translation2d(kBackLeftXPosInches, kBackLeftYPosInches),
    };

    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    @Override
    public SwerveDrivetrainConstants getIds() {
        return DrivetrainConstants;
    }

    @Override
    public double getMaxSpeedMetersPerSecond() {
        return MaxSpeed;
    }

    @Override
    public double getMaxAngularRateRadiansPerSecond() {
        return MaxAngularRate;
    }

    @Override
    public SwerveModuleConstants getFrontLeft() {
        return FrontLeft;
    }

    @Override
    public SwerveModuleConstants getFrontRight() {
        return FrontRight;
    }

    @Override
    public SwerveModuleConstants getBackLeft() {
        return BackLeft;
    }

    @Override
    public SwerveModuleConstants getBackRight() {
        return BackRight;
    }
}
