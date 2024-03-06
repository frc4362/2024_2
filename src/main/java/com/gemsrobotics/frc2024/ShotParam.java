package com.gemsrobotics.frc2024;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShotParam {
    private Rotation2d m_angle;
    private double m_velocityRps;

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
}
