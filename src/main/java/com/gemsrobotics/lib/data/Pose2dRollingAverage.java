package com.gemsrobotics.lib.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Pose2dRollingAverage {
    private List<Pose2d> m_rollingList = new ArrayList<Pose2d>();
    private int m_size = 0;

    public Pose2dRollingAverage(int size) {
        m_size = size;
    }

    public void add(Pose2d item) {
        if (m_rollingList.size() == m_size) {
            m_rollingList.remove(0);
        }
        m_rollingList.add(item);
    }

    public Optional<Pose2d> getAverage() {
        Optional<Pose2d> out = Optional.empty();

        if (m_rollingList.size() == m_size) {
            Translation2d transSum = new Translation2d();
            Rotation2d rotSum = new Rotation2d();
            for (int i = 0; i < m_size; i++) {
                transSum = transSum.plus(m_rollingList.get(i).getTranslation());
                rotSum = rotSum.plus(m_rollingList.get(i).getRotation());
            }
            transSum = transSum.div(m_size);
            rotSum = rotSum.div(m_size);
            out = Optional.of(new Pose2d(transSum, rotSum));
        }

        return out;
    }
}
