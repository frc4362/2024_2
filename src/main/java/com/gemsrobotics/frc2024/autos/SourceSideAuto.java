package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootCommand;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SourceSideAuto extends SequentialCommandGroup {
	public SourceSideAuto() {
		final var drive = Swerve.getInstance();
		final var driveToFirstShot = drive.getTrackTrajectoryCommand("Source Side.1", true);
		final var driveToSecondShot = drive.getTrackTrajectoryCommand("Source Side.2", false);
		final var driveTo3rdShot = drive.getTrackTrajectoryCommand("Source Side.3", false);
		final var driveTo4thShot = drive.getTrackTrajectoryCommand("Source Side.4", false);
		final var driveTo5thShot = drive.getTrackTrajectoryCommand("Source Side.5", false);

		addCommands(
				driveToFirstShot,
				new ShootCommand(1.1, true, true),
				new SetIntakeForcedOutCommand(true),
				new ParallelDeadlineGroup(
						driveToSecondShot,
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new ShootCommand(1.1, true, true),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				driveTo3rdShot,
				new ShootCommand(1.1, true, true),
				driveTo4thShot,
				new ShootCommand(1.1, true, true),
				new SetIntakeForcedOutCommand(false),
				driveTo5thShot
		);
	}
}
