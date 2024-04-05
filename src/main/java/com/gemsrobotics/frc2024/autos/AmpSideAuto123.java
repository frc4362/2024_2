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
import edu.wpi.first.wpilibj2.command.*;

public class AmpSideAuto123 extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Amp 123";
	public AmpSideAuto123() {
		final var drive = Swerve.getInstance();
		final var pathToFirstShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + " 1" + ".1", true);
		final var pathToSecondShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + " 1" + ".2", false);
		final var pathToPickup = drive.getTrackTrajectoryCommand(AUTO_NAME + " 1" + ".3", false);
		final var pathToThirdShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".1", false);
		final var pathToFourthShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".2", false);
		final var pathToFifthShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".3", false);

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
				pathToPickup,
				new WaitCommand(0.1),
				pathToThirdShootLocation,
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToFourthShootLocation,
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToFifthShootLocation,
				new ShootNoteCommand(2.0, true).onlyIf(() -> Fintake.getInstance().isHoldingPiece()),
				new SetIntakeForcedOutCommand(false)
		);
	}
}
