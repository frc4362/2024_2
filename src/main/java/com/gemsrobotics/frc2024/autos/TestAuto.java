package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestAuto extends SequentialCommandGroup {
	public TestAuto() {
		final var drive = Swerve.getInstance();
		final var path = drive.getTrackTrajectoryCommand("Test Traj", true);

		addCommands(path);
	}
}
