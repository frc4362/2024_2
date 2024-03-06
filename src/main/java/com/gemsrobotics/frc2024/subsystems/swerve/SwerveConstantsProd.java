package com.gemsrobotics.frc2024.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import com.gemsrobotics.frc2024.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class SwerveConstantsProd implements SwerveConstants {
    private static final double kKrakenRadiansPerSecond = DCMotor.getKrakenX60(1).freeSpeedRadPerSec;
    private static final double kWheelRadiusInches = 1.91555; //4.0 / 2.0; // calipered

    private static final double kDriveGearRatio = 4.59375;
    private static final double kSteerGearRatio = 13.371428571428572;

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);

    private static final double TRANSMISSION_EFFICIENCY = 0.95;

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3.0).withKI(0).withKD(0)
        .withKS(0.3) // tuned with controller lol
        // analytic one is 12.0 / (kKrakenRadiansPerSecond / (Math.PI * 2) / kDriveGearRatio) * TRANSMISSION_EFFICIENCY
        .withKV(.55)
        .withKA(0.015);
    // analytically derived kV from motor properties

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 60.0;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
//    public static final double kSpeedAt12VoltsMps = 7.145; // analytically derived below
    public static final double kSpeedAt12VoltsMps = kKrakenRadiansPerSecond / kDriveGearRatio * Units.inchesToMeters(kWheelRadiusInches);

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = Constants.MAIN_BUS_NAME;
    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

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
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 4;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = -0.4658203125;

    private static final double kFrontLeftXPosInches = 10.125;
    private static final double kFrontLeftYPosInches = 10.125;

    // Front Right
    private static final int kFrontRightDriveMotorId = 6;
    private static final int kFrontRightSteerMotorId = 8;
    private static final int kFrontRightEncoderId = 9;
    private static final double kFrontRightEncoderOffset = -0.40966796875;

    private static final double kFrontRightXPosInches = 10.125;
    private static final double kFrontRightYPosInches = -10.125;

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 7;
    private static final int kBackLeftEncoderId = 11;
    private static final double kBackLeftEncoderOffset = -0.08203125;

    private static final double kBackLeftXPosInches = -10.125;
    private static final double kBackLeftYPosInches = 10.125;

    // Back Right
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 5;
    private static final int kBackRightEncoderId = 10;
    private static final double kBackRightEncoderOffset = -0.158447265625;

    private static final double kBackRightXPosInches = -10.125;
    private static final double kBackRightYPosInches = -10.125;

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
        return kSpeedAt12VoltsMps;
    }

    @Override
    public double getMaxAngularRateRadiansPerSecond() {
        return 9.55;
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
