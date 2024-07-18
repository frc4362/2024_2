package com.gemsrobotics.lib.allianceconstants;

import com.gemsrobotics.lib.math.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class AllianceConstantsBlue extends AllianceConstants {
    public static final Translation2d AMP_FEED_OFFSET = new Translation2d(1.0, -0.4);
    public static final Translation2d AMP_TO_MID_OFFSET = new Translation2d(4.0, 0.0);

    public static final Rotation2d BLUE_NORTH = Rotation2d.fromDegrees(0);
    public static final Rotation2d BLUE_SOUTH = Rotation2d.fromDegrees(180);
    public static final Rotation2d BLUE_EAST = Rotation2d.fromDegrees(270);
    public static final Rotation2d BLUE_WEST = Rotation2d.fromDegrees(90);
    public static final Translation2d BLUE_SPEAKER_METERS = new Translation2d(0., 5.5); // Approximated from Choreo points
    public static final Translation2d BLUE_AMP_METERS = new Translation2d(1.85, 8.15); // Approximated from Choreo points
    public static final Translation2d BLUE_SOURCE_METERS = new Translation2d(15.6, 0.575); //Approximated from Choreo points
    public static final Translation2d BLUE_AMP_FEED_LOCATION_METERS = BLUE_AMP_METERS.plus(AMP_FEED_OFFSET);
    public static final Translation2d BLUE_MID_FEED_LOCATION_METERS = BLUE_AMP_FEED_LOCATION_METERS.plus(AMP_TO_MID_OFFSET);

    public AllianceConstantsBlue() {}


    @Override
    public Rotation2d getNorth() {
        return BLUE_NORTH;
    }

    @Override
    public Rotation2d getSouth() {
        return BLUE_SOUTH;
    }

    @Override
    public Rotation2d getEast() {
        return BLUE_EAST;
    }

    @Override
    public Rotation2d getWest() {
        return BLUE_WEST;
    }

    @Override
    public Translation2d getSpeakerLocationMeters() {
        return BLUE_SPEAKER_METERS;
    }

    @Override
    public Translation2d getAmpLocationMeters() {
        return BLUE_AMP_METERS;
    }

    @Override
    public Translation2d getAmpZoneLocationMeters() {
        return BLUE_AMP_FEED_LOCATION_METERS;
    }

    @Override
    public Translation2d getMiddleFeedLocationMeters() {
        return BLUE_MID_FEED_LOCATION_METERS;
    }

    @Override
    public Translation2d getSourceLocationMeters() {
        return BLUE_SOURCE_METERS;
    }

    @Override
    public boolean isLocationInOpposingWing(final Translation2d location) {
        return location.getX() > 11.25;
    }
}
