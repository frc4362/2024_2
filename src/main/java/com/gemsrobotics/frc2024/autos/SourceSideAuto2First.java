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

public class SourceSideAuto2First extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Source 1+0+3 B";
	public SourceSideAuto2First() {
		final var drive = Swerve.getInstance();
		final var driveToFirstShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".1", true);
		final var driveToSecondShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".2", false);
		final var driveTo3rdShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".3", false);
		final var driveTo4thShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".4", false);
//		final var driveToMiddle = drive.getTrackTrajectoryCommand(AUTO_NAME + ".5", false);

		addCommands(
				driveToFirstShot,
				new ShootNoteCommand(2.0, true),
				new SetIntakeForcedOutCommand(true),
				new ParallelDeadlineGroup(
						driveToSecondShot,
						new SequentialCommandGroup(
								new WaitCommand(0.1),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new WaitCommand(1.0),
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				driveTo3rdShot,
				new SetIntakeForcedOutCommand(false),
//				new ShootNoteCommand(3.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				driveTo4thShot
//				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
//				new SetIntakeForcedOutCommand(false),
//				driveToMiddle
		);
	}
}
