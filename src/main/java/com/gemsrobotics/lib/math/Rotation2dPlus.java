package com.gemsrobotics.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Rotation2dPlus extends Rotation2d {
	public Rotation2dPlus() {
		super();
	}

	public Rotation2dPlus(final double radians) {
		super(radians);
	}

	public Rotation2dPlus(final Rotation2d rotation) {
		super(rotation.getRadians());
	}

	public Rotation2dPlus(final double x, final double y) {
		super(x, y);
	}

	public static Rotation2dPlus fromRadians(final double radians) {
		return new Rotation2dPlus(radians);
	}

	public static Rotation2dPlus fromDegrees(final double degrees) {
		return fromRadians(Units.degrees2Rads(degrees));
	}

	public static Rotation2dPlus fromRotations(final double rotations) {
		return fromRadians(rotations * 2 * Math.PI);
	}

	/**
	 * @return The pole nearest to this rotation.
	 */
	public Rotation2dPlus getNearestPole() {
		final double poleSin;
		final double poleCos;

		if (Math.abs(getCos()) > Math.abs(getSin())) {
			poleCos = Math.signum(getCos());
			poleSin = 0.0;
		} else {
			poleCos = 0.0;
			poleSin = Math.signum(getSin());
		}

		return new Rotation2dPlus(poleCos, poleSin);
	}

	public Rotation2dPlus getNearestStagePole() {
		final double poleSin;
		final double poleCos;

		if (Math.abs(getCos()) > Math.abs(getSin())) {
			poleCos = Math.signum(getCos());
			poleSin = 0.0;
		} else {
			poleCos = 0.0;
			poleSin = Math.signum(getSin());
		}

		return new Rotation2dPlus(poleCos, poleSin);
	}
}
