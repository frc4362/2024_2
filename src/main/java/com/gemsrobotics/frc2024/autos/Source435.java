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

public class Source435 extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Source 435";

	public Source435() {
		final var drive = Swerve.getInstance();
		final var turnToFirstShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".1", true);
		final var driveTo1stShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".2", false);
		final var driveTo2ndShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".3", false);
		final var driveTo3rdShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".4", false);
		final var driveToMiddle = drive.getTrackTrajectoryCommand(AUTO_NAME + ".5", false);

		addCommands(
				drive.resetOdometryOnTrajectory(AUTO_NAME + ".1"),
				new ShootNoteCommand(2.0, true),
				new SetIntakeForcedOutCommand(true),
				new ParallelDeadlineGroup(
						driveTo1stShot,
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new ParallelDeadlineGroup(
						driveTo2ndShot,
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
				new SetIntakeForcedOutCommand(false),
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.IDLE),
				driveToMiddle
		);
	}
}
