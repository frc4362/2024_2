package com.gemsrobotics.frc2024.subsystems;

import com.gemsrobotics.frc2024.ShotParam;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class ShotLogger {
    private DataLog m_log;
    private StringLogEntry m_shotLog;
    private int m_shotCount;

    public ShotLogger() {
        DataLogManager.start();

        m_log = DataLogManager.getLog();
        m_shotLog = new StringLogEntry(m_log, "/shots");
        m_shotCount = 0;
    }

    public void logShot(final ShotParam requested, double shotSpeed, Rotation2d shotAngle, double shotDistance) {
        String str = "{requested = " + requested.getVelocityRps() + ", angle = " + requested.getAngle().toString() + "; actual: v = " + shotSpeed + ", angle = " + shotAngle.toString() + "; @ a distance of " + shotDistance;
        m_shotLog.append(str);
    }
}
