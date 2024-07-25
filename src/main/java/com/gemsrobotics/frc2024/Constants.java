package com.gemsrobotics.frc2024;

import com.gemsrobotics.lib.allianceconstants.AllianceConstants;
import com.gemsrobotics.lib.allianceconstants.AllianceConstantsBlue;
import com.gemsrobotics.lib.allianceconstants.AllianceConstantsRed;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

public final class Constants {
	public static final String MAIN_BUS_NAME = "swerve";
	public static final String AUX_BUS_NAME = "aux";

	public static final boolean DO_TRAP_CONFIGURATION = true;

	private Constants() {
	}

	public static final AllianceConstants ALLIANCE_CONSTANTS_BLUE = new AllianceConstantsBlue();
	public static final AllianceConstants ALLIANCE_CONSTANTS_RED = new AllianceConstantsRed();
	public static final AllianceConstants ALLIANCE_CONSTANTS_DEFAULT = ALLIANCE_CONSTANTS_BLUE;

	public static AllianceConstants getAllianceConstants() {
		final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isEmpty()) {
			return ALLIANCE_CONSTANTS_DEFAULT;
		} else if (alliance.get() == DriverStation.Alliance.Blue) {
			return ALLIANCE_CONSTANTS_BLUE;
		} else {
			return ALLIANCE_CONSTANTS_RED;
		}
	}

	public static final int PILOT_JOYSTICK_PORT = 0;
	public static final int COPILOT_JOYSTICK_PORT = 1;

	public static final String LIMELIGHT_SHOOTER_KEY = "";

	// distance, angle, velocity
	private static final double[][] SHOT_CALS = {
			// first cal is subwoofer shot

			{1.3, 42.5, 95},
			{1.66, 32.0, 120.0},
			{1.95, 29.0, 120.0},
			{2.1, 27.25, 120.0},
			{2.22, 24.8, 120.0},
			{2.45, 23.5, 120.0},
			{2.71, 20.8, 120.0},
			{2.97, 18.0, 120.0},
			{3.52, 14.75, 120.0},
			{3.95, 13.75, 120.0},
			{4.25, 12.5, 120.0},
			{4.53, 11.8, 120.0},
			{4.8, 10.5, 120.0},
			{4.99, 10.2, 120.0},
			{5.18, 9.8, 120.0},
			{5.3, 9.3, 120.0},
			{5.5, 9.1, 120.0},
			{5.8, 8.9, 120.0},
	};

//	{1.3, 42.5, 95},
//	{1.66, 32.0, 120.0},
//	{1.95, 29.0, 120.0},
//	{2.1, 27.25, 120.0},
//	{2.22, 24.8, 120.0},
//	{2.45, 23.5, 120.0},
//	{2.71, 20.8, 120.0},
//	{2.97, 18.0, 120.0},
//	{3.52, 14.75, 120.0},
//	{3.95, 13.75, 120.0},
//	{4.25, 12.5, 120.0},
//	{4.53, 11.1, 120.0},
//	{4.8, 9.9, 120.0},
//	{4.99, 9.5, 120.0},
//	{5.18, 9.25, 120.0},
//	{5.3, 8.9, 120.0},
//	{5.5, 8.7, 120.0},
//	{5.8, 8.5, 120.0},

	private static double SHOT_FLAT_ADJUSTMENT = 2.0;//2.0;

	private static final ShotParam MIN_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[0][1]), SHOT_CALS[0][2]);
	private static final ShotParam MAX_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[SHOT_CALS.length - 1][1]), SHOT_CALS[SHOT_CALS.length - 1][2]);

	private static final InterpolatingTreeMap<Double,ShotParam> SHOT_PARAMETERS = new InterpolatingTreeMap<Double,ShotParam>(InverseInterpolator.forDouble(), new ShotParamInterpolator());
	static {
		for (final double[] cals : SHOT_CALS) {
			if (cals.length != 3) {
				continue;
			}

			SHOT_PARAMETERS.put(cals[0], new ShotParam(Rotation2d.fromDegrees(cals[1]), cals[2]));
		}
	}

	public static ShotParam getShotParameters(final double distanceMeters) {
		final ShotParam ret;

		// first cal is subwoofer shot
		if (distanceMeters < SHOT_CALS[1][0]) {
			ret = MIN_SHOT;
		} else if (distanceMeters > SHOT_CALS[SHOT_CALS.length - 1][0]) {
			ret = MAX_SHOT;
		} else {
			ret = SHOT_PARAMETERS.get(distanceMeters);
		}

		return new ShotParam(ret.getAngle().plus(Rotation2d.fromDegrees(SHOT_FLAT_ADJUSTMENT)), ret.getVelocityRps());
	}

	// distance, angle, velocity
	private static final double[][] FEED_CALS = {
			{7.0, 36.0, 62.0},
			{7.3, 35.0, 64.0},
			{8.0, 35.0, 66.5},
			{9.5, 35.0, 69.5},
			{11.0, 32.0, 72.5},
	};

	private static final ShotParam FEED_MIN_SHOT = new ShotParam(Rotation2d.fromDegrees(FEED_CALS[0][1]), FEED_CALS[0][2]);
	private static final ShotParam FEED_MAX_SHOT = new ShotParam(Rotation2d.fromDegrees(FEED_CALS[FEED_CALS.length - 1][1]), FEED_CALS[FEED_CALS.length - 1][2]);

	private static final InterpolatingTreeMap<Double,ShotParam> FEED_SHOT_PARAMETERS = new InterpolatingTreeMap<Double,ShotParam>(InverseInterpolator.forDouble(), new ShotParamInterpolator());
	static {
		for (final double[] cals : FEED_CALS) {
			if (cals.length != 3) {
				continue;
			}

			FEED_SHOT_PARAMETERS.put(cals[0], new ShotParam(Rotation2d.fromDegrees(cals[1]), cals[2]));
		}
	}

	public static ShotParam getFeedParameters(final double distanceMeters) {
		ShotParam ret;

		// first cal is subwoofer shot
		if (distanceMeters < SHOT_CALS[1][0]) {
			ret = FEED_MIN_SHOT;
		} else if (distanceMeters > SHOT_CALS[SHOT_CALS.length - 1][0]) {
			ret = FEED_MAX_SHOT;
		} else {
			ret = FEED_SHOT_PARAMETERS.get(distanceMeters);
		}

		return new ShotParam(ret.getAngle(), ret.getVelocityRps());
	}

	public static void adjustShots(final double degrees) {
		SHOT_FLAT_ADJUSTMENT += degrees;
	}

	public static final double MAX_SUGGESTED_RANGE_METERS = 5.3;
	public static final double LED_SHUTOFF_RANGE_METERS = 8.0;
}
