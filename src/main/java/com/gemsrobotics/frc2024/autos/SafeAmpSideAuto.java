package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootCommand;
import com.gemsrobotics.frc2024.commands.ShootNoteCommand;
import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.Shooter;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;

public class SafeAmpSideAuto extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Amp SAFE";
	public SafeAmpSideAuto() {
		final var drive = Swerve.getInstance();
		final var pathToFirstShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".1", true);
		final var pathToSecondShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".2", false);
		final var pathToThirdShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".3", false);
		final var pathToFourthShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".4", false);
		final var pathToFifthShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".5", false);

		addCommands(
				new SetIntakeForcedOutCommand(true),
				pathToFirstShootLocation,
				new ShootNoteCommand(5.0, false),
				new InstantCommand(() -> Superstructure.getInstance().setWantsIntaking(true)),
				pathToSecondShootLocation,
				new ShootCommand(0.5, true, false),
				new InstantCommand(() -> Shooter.getInstance().setDoIdling(false)),
				new InstantCommand(() -> Superstructure.getInstance().setWantsIntaking(false)),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				new ParallelDeadlineGroup(
						pathToThirdShootLocation,
						new SequentialCommandGroup(
								new WaitCommand(2.0),
								new InstantCommand(() -> Shooter.getInstance().setDoIdling(false))
						)
				).andThen(new InstantCommand(() -> Shooter.getInstance().setDoIdling(false))),
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToFourthShootLocation,
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToFifthShootLocation,
				new SetIntakeForcedOutCommand(false)
		);
	}
}
