package com.gemsrobotics.frc2024.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

public interface SwerveConstants {
	SwerveDrivetrainConstants getIds();
	double getMaxSpeedMetersPerSecond();
	double getMaxAngularRateRadiansPerSecond();
	SwerveModuleConstants getFrontLeft();
	SwerveModuleConstants getFrontRight();
	SwerveModuleConstants getBackLeft();
	SwerveModuleConstants getBackRight();
}
