package com.gemsrobotics.lib;

import com.ctre.phoenix6.hardware.TalonFX;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

/*
 * This class is used to contain various helpful constants and functions for using TalonFX motors
 * 
 */
public class TalonUtils {
    public static final int CONFIG_MOTOR_TRIES = 10;

    
    //attempts to configure the given motor with the given TalonFXConfiguration a given number of times.
    public static void configureTalon(TalonFXConfiguration config, TalonFX motor) {
        for (int i = TalonUtils.CONFIG_MOTOR_TRIES; i > 0; i--) {
            if (motor.getConfigurator().apply(config) == StatusCode.OK) {
                return;
            }
        }
        System.err.println("Failed to configure motor");
    }

    //sets the follower motor to follow the leader motor
    public static void setFollower(TalonFX follower, TalonFX leader, boolean invertMotion) {
        follower.setControl(new Follower(leader.getDeviceID(), invertMotion));
    }
}
