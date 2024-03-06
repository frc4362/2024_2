package com.gemsrobotics;

import com.gemsrobotics.frc2024.ShotParam;
import com.gemsrobotics.frc2024.ShotParamInterpolator;

import edu.wpi.first.math.geometry.Rotation2d;

public class InterpolationTests {
    private InterpolationTests() {

    }

    public static void main(String... args) {

        ShotParamInterpolator interpolator = new ShotParamInterpolator();

        ShotParam testVal1 = new ShotParam(Rotation2d.fromRadians(0), 0.0);
        ShotParam testVal2 = new ShotParam(Rotation2d.fromRadians(2), 1.0);

        ShotParam goodResult = new ShotParam(Rotation2d.fromRadians(1.0), 0.5);

        assert interpolator.interpolate(testVal1, testVal2, 0.5) == goodResult;
    }
}
