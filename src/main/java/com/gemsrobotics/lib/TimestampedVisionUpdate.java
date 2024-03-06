package com.gemsrobotics.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import java.util.List;

// Based on https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/apriltagvision/AprilTagVision.java#L261
public final class TimestampedVisionUpdate {
    private static final double STD_DEV_COEFFICIENT = 0.12; // meters
    private static final double THETA_STD_DEV_COEFFICIENT = 2; // radians

    private final double m_timestampSeconds;
    private final Pose2d m_robotPose; //where the robot thinks it is (based on limelight data)
    private final List<Pose2d> m_camToTagPoses; // where robot thinks the tag is relative to its camera
    private final Vector<N3> m_stdDevVector;

    public TimestampedVisionUpdate(double timestampSeconds, Pose2d robotPose, List<Pose2d> camToTagPoses, List<Double> tagDistancesMeters) {
        this(timestampSeconds, robotPose, camToTagPoses, getStdDevVector(tagDistancesMeters));
    }

    public TimestampedVisionUpdate(double timestampSeconds, Pose2d robotPose, List<Pose2d> camToTagPoses, Vector<N3> stdDevVector) {
        m_timestampSeconds = timestampSeconds;
        m_robotPose = robotPose;
        m_camToTagPoses = camToTagPoses;
        m_stdDevVector = stdDevVector;
    }

// Pass in array of getTargetPose_CameraSpace2d() results
    private static Vector<N3> getStdDevVector(final List<Double> tagDistancesMeters) {
        double totalDistanceMeters = 0;
        for (double d : tagDistancesMeters) {
            totalDistanceMeters += d;
        }
        double avgDistanceMeters = totalDistanceMeters / tagDistancesMeters.size();
        double stdDev = STD_DEV_COEFFICIENT * Math.pow(avgDistanceMeters, 2.0) / tagDistancesMeters.size();
        return VecBuilder.fill(stdDev, stdDev, THETA_STD_DEV_COEFFICIENT);
    }

    public double getTimestampSeconds() {
        return m_timestampSeconds;
    }

    public Pose2d getRobotPose() {
        return m_robotPose;
    }

    public List<Pose2d> getCamToTagPoses() {return m_camToTagPoses;}

    public Vector<N3> getExpectedError() {
        return m_stdDevVector;
    }
}