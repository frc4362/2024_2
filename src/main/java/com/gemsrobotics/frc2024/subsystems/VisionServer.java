package com.gemsrobotics.frc2024.subsystems;

import com.gemsrobotics.lib.Limelight;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Objects;

public final class VisionServer extends Limelight implements Subsystem {
	private static VisionServer INSTANCE = null;

	public static VisionServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new VisionServer();
		}

		return INSTANCE;
	}

	private VisionServer() {
		super();
	}
}
