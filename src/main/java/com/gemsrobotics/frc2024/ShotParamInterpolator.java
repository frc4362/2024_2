package com.gemsrobotics.frc2024;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;

public class ShotParamInterpolator implements Interpolator<ShotParam> {

    @Override
    public ShotParam interpolate(ShotParam startValue, ShotParam endValue, double t) {
        return new ShotParam(startValue.getAngle().interpolate(endValue.getAngle(), t), MathUtil.interpolate(startValue.getVelocityRps(), endValue.getVelocityRps(), t));
    }
    
}
