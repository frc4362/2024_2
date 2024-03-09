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

	public static final boolean USE_PREMATCH_LOCALIZATION = false;

	public static final double WINCH_FULLY_EXTENDED = 5.0;
	public static final double WINCH_FULLY_RETRACTED = 0.0;

	public static final int PILOT_JOYSTICK_PORT = 0;
	public static final int COPILOT_JOYSTICK_PORT = 1;

	public static final String LIMELIGHT_SHOOTER_KEY = "";

	// distance, angle, velocity
	private static final double[][] SHOT_CALS = {
			// first cal is subwoofer shot

			{0.0, 42.5, 120},
			{1.96, 25, 120},
			{2.35, 20, 120},
			{2.67, 18.1, 120},
			{3.00, 15.75, 120},
			{3.55, 13.5, 120},
			{3.71, 12.0, 120},
			{4.11, 10.5, 120},
			{4.55, 9.35, 120},
			{5.2, 7.85, 120},
			{6.08, 6.87, 120},
			{6.8, 6.1, 120},
	};

	private static final ShotParam MIN_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[0][1]), SHOT_CALS[0][2] + 2);
	private static final ShotParam MAX_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[SHOT_CALS.length - 1][1]), SHOT_CALS[SHOT_CALS.length - 1][2] + 2);

	private static InterpolatingTreeMap<Double,ShotParam> SHOT_PARAMETERS = new InterpolatingTreeMap<Double,ShotParam>(InverseInterpolator.forDouble(), new ShotParamInterpolator());
	static {
		for (final double[] cals : SHOT_CALS) {
			if (cals.length != 3) {
				continue;
			}

			SHOT_PARAMETERS.put(cals[0], new ShotParam(Rotation2d.fromDegrees(cals[1]), cals[2]+5));
		}
	}

	public static ShotParam getShotParameters(final double distanceMeters) {
		// first cal is subwoofer shot
		if (distanceMeters < SHOT_CALS[1][0]) {
			return MIN_SHOT;
		} else if (distanceMeters > SHOT_CALS[SHOT_CALS.length - 1][0]) {
			return MAX_SHOT;
		} else {
			return SHOT_PARAMETERS.get(distanceMeters);
		}
	}

	public static final double MAX_SUGGESTED_RANGE_METERS = 4.35;
	public static final double LED_SHUTOFF_RANGE_METERS = 8.0;

	public static final PIDController SWERVE_TURN_CONTROLLER = new PIDController(5.0, 0, 0);
	static {
		SWERVE_TURN_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
	}

	public static final String MAIN_BUS_NAME = "swerve";
	public static final String AUX_BUS_NAME = "aux";
}
