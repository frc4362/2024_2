package com.gemsrobotics.lib.allianceconstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AllianceConstantsRed extends AllianceConstants {

    public static final Rotation2d RED_NORTH = Rotation2d.fromDegrees(180);
    public static final Rotation2d RED_SOUTH = Rotation2d.fromDegrees(0);
    public static final Rotation2d RED_EAST = Rotation2d.fromDegrees(90);
    public static final Rotation2d RED_WEST = Rotation2d.fromDegrees(270);
    public static final Translation2d RED_SPEAKER_METERS =new Translation2d(16.5, 5.5); // Approximated from Choreo points
    public static final Translation2d RED_AMP_METERS = new Translation2d(14.7, 8.15); // Approximated from Choreo points
    public static final Translation2d RED_SOURCE_METERS = new Translation2d(0.95, 0.575); //Approximated from Choreo points

    public AllianceConstantsRed() {}

    @Override
    public Rotation2d getNorth() {
        return RED_NORTH;
    }

    @Override
    public Rotation2d getSouth() {
        return RED_SOUTH;
    }

    @Override
    public Rotation2d getEast() {
        return RED_EAST;
    }

    @Override
    public Rotation2d getWest() {
        return RED_WEST;
    }

    @Override
    public Translation2d getSpeakerLocationMeters() {
        return RED_SPEAKER_METERS;
    }

    @Override
    public Translation2d getAmpLocationMeters() {
        return RED_AMP_METERS;
    }

    @Override
    public Translation2d getSourceLocationMeters() {
        return RED_SOURCE_METERS;
    }
}
