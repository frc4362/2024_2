package com.gemsrobotics.lib.allianceconstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class AllianceConstants {
    public abstract Rotation2d getNorth();
    public abstract Rotation2d getSouth();
    public abstract Rotation2d getEast();
    public abstract Rotation2d getWest();
    public abstract Translation2d getSpeakerLocationMeters();
    public abstract Translation2d getAmpLocationMeters();
    public abstract Translation2d getSourceLocationMeters();
}
