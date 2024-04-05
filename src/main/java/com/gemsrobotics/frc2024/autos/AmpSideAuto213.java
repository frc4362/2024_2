package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootCommand;
import com.gemsrobotics.frc2024.commands.ShootNoteCommand;
import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.Shooter;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AmpSideAuto213 extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Amp 213";
	public AmpSideAuto213() {
		final var drive = Swerve.getInstance();
		final var pathToFirstShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".1", true);
		final var pathToSecondShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".2", false);
		final var pathToFirstCenter = drive.getTrackTrajectoryCommand(AUTO_NAME + ".3", false);
		final var pathToSecondCenter = drive.getTrackTrajectoryCommand(AUTO_NAME + ".4", false);
		final var pathToThirdCenter = drive.getTrackTrajectoryCommand(AUTO_NAME + ".5", false);

		final double MOVING_SHOT_ADJUSTMENT = -2.5;

		addCommands(
				new SetIntakeForcedOutCommand(true),
				pathToFirstShootLocation,
				new ShootNoteCommand(5.0, false),
				new InstantCommand(() -> Superstructure.getInstance().setWantsIntaking(true)),
				new InstantCommand(() -> Constants.adjustShots(MOVING_SHOT_ADJUSTMENT)),
				pathToSecondShootLocation,
				new ShootCommand(0.45, true, false),
				new InstantCommand(() -> Constants.adjustShots(-MOVING_SHOT_ADJUSTMENT)),
				new InstantCommand(() -> Shooter.getInstance().setDoIdling(false)),
				new InstantCommand(() -> Superstructure.getInstance().setWantsIntaking(false)),
				new SetIntakeForcedOutCommand(false),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToFirstCenter,
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToSecondCenter,
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToThirdCenter,
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetIntakeForcedOutCommand(false)
		);
	}
}
