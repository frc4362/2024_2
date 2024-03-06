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
//			{ 0.0, 56.0, 90.0},
//			{ 1.7, 44.0, 95.0},
//			{ 1.84, 42.0, 105.0},
//			{ 2.5, 35, 116.0},
//			{ 3.34, 28.5, 116.0},
//			{ 4.34, 28.5, 92.0},
//			{ 5, 28.5, 85.0},

//			{ 1.0, 58, 100 },
//			{ 1.4, 51.25, 100 },
//			{ 1.59, 49.0, 100 },
//			{ 2.00, 45.5, 100 },
//			{ 2.32, 41.25, 100 },
//			{ 2.52, 39.7, 100 },
//			{ 2.87, 36.5, 100 },
//			{ 3.33, 33.5, 100 },
//			{ 3.80, 28.5, 96 },
//			{ 4.11, 28.5, 87 },
//			{ 4.77, 28.5, 81 },
//			{ 5.11, 28.5, 80 }

			{ 1.0, 58, 100 },
			{ 1.4, 51.25, 100 },
			{ 1.59, 49.0, 100 },
			{ 2.00, 47.5, 100 },
			{ 2.32, 43.87, 100 },
			{ 2.52, 40.8, 100 },
			{ 2.73, 40.4, 100 },
			{ 2.87, 37.9, 100 },
//			{ 3.00, 37.4, 100 },
			{ 3.33, 35.0, 100 },
			{ 3.80, 28.5, 96 },
			{ 4.11, 28.5, 87 },
			{ 4.77, 28.5, 81 },
			{ 5.11, 28.5, 80 }

//			{ 1.0, 60, 100 },
//			{ 1.4, 55.25, 100 },
//			{ 1.59, 53.5, 100 },
//			{ 2.00, 49.5, 100 },
//			{ 2.32, 42.35, 100 },
//			{ 2.52, 40.3, 100 },
//			{ 2.72, 39.1, 100 },
//			{ 2.87, 37., 100 },
//			{ 3.33, 34.0, 100 },
//			{ 3.5, 32.0, 100 },
//			{ 3.80, 29.0, 96 },
//			{ 4.11, 28.5, 87 },
//			{ 4.77, 28.5, 81 },
//			{ 5.11, 28.5, 80 }
	};

	private static final ShotParam MIN_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[0][1]), SHOT_CALS[0][2] + 5);
	private static final ShotParam MAX_SHOT = new ShotParam(Rotation2d.fromDegrees(SHOT_CALS[SHOT_CALS.length - 1][1]), SHOT_CALS[SHOT_CALS.length - 1][2] + 5);

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
