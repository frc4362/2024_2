package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootNoteCommand;
import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SourceSideAuto1First extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Source 123";
	public SourceSideAuto1First() {
		final var drive = Swerve.getInstance();
		final var driveToFirstShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 1" + ".1", true);
		final var driveToPickup = drive.getTrackTrajectoryCommand(AUTO_NAME + " 1" + ".2", false);
		final var driveTo2ndShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".1", false);
		final var driveTo3rdShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2"+ ".2", false);
		final var driveTo4thShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".3", false);
		final var driveToMiddle = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".4", false);

		addCommands(
				drive.resetOdometryOnTrajectory(AUTO_NAME + " 1" + ".1"),
				new ShootNoteCommand(3.0, true),
				new SetIntakeForcedOutCommand(true),
				new ParallelDeadlineGroup(
						new SequentialCommandGroup(
								driveToPickup,
								new WaitCommand(0.1), // bouncing lol
								driveTo2ndShot
						),
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new ParallelDeadlineGroup(
						driveTo3rdShot,
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new ParallelDeadlineGroup(
						driveTo4thShot,
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new SetIntakeForcedOutCommand(false),
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.IDLE),
				driveToMiddle
		);
	}
}
