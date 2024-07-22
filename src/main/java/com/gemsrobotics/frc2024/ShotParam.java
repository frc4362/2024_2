package com.gemsrobotics.frc2024;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ShotParam {
    private final Rotation2d m_angle;
    private final double m_velocityRps;

    public ShotParam(Rotation2d angle, double velocity) {
        m_angle = angle;
        m_velocityRps = velocity;
    }

    public Rotation2d getAngle() {
        return m_angle;
    }

    public double getVelocityRps() {
        return m_velocityRps;
    }

    @Override
    public String toString() {
        return String.format("(%.2f deg, %.2f rps)", m_angle.getDegrees(), m_velocityRps);
    }
}
