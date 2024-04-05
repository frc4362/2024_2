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
			{2.71, 20.4, 120.0},
			{2.97, 16.7, 120.0},
			{3.52, 13.8, 120.0},
			{3.95, 12.8, 120.0},
			{4.25, 11.8, 120.0},
			{4.53, 11.1, 120.0},
			{4.8, 10.75, 120.0},
			{4.99, 10.25, 120.0},
			{5.18, 9.65, 120.0},
			{5.3, 9.1, 120.0},
			{5.5, 8.8, 120.0},
			{5.8, 8.5, 120.0},
	};

	private static double SHOT_FLAT_ADJUSTMENT = 3.5;//2.0;

	private static final ShotParam MIN_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[0][1]), SHOT_CALS[0][2]);
	private static final ShotParam MAX_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[SHOT_CALS.length - 1][1]), SHOT_CALS[SHOT_CALS.length - 1][2]);

	private static InterpolatingTreeMap<Double,ShotParam> SHOT_PARAMETERS = new InterpolatingTreeMap<Double,ShotParam>(InverseInterpolator.forDouble(), new ShotParamInterpolator());
	static {
		for (final double[] cals : SHOT_CALS) {
			if (cals.length != 3) {
				continue;
			}

			SHOT_PARAMETERS.put(cals[0], new ShotParam(Rotation2d.fromDegrees(cals[1]), cals[2]));
		}
	}

	public static ShotParam getShotParameters(final double distanceMeters) {
		ShotParam ret;

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

	private static final ShotParam FLAT_SHOT = new ShotParam(Rotation2d.fromDegrees(0.0), 120.0);
	public static ShotParam getFlatShot() {
		return FLAT_SHOT;
	}

	public static void adjustShots(final double degrees) {
		SHOT_FLAT_ADJUSTMENT += degrees;
	}

	public static final double MAX_SUGGESTED_RANGE_METERS = 5.3;
	public static final double LED_SHUTOFF_RANGE_METERS = 8.0;

	public static final PIDController SWERVE_TURN_CONTROLLER = new PIDController(5.0, 0, 0);
	static {
		SWERVE_TURN_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
	}

	public static final String MAIN_BUS_NAME = "swerve";
	public static final String AUX_BUS_NAME = "aux";
}
