package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootCommand;
import com.gemsrobotics.frc2024.commands.ShootNoteCommand;
import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.Shooter;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.*;

public class CenterAuto extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Close Auto";
	public CenterAuto() {
		final var drive = Swerve.getInstance();
		final var pathToFirstShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".1", true);
		final var pathToSecondShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".2", false);
		final var pathToThirdShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".3", false);
		final var pathToFourthShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".4", false);
		final var pathToFifthShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".5", false);

		addCommands(
				new SetIntakeForcedOutCommand(false),
				drive.resetOdometryOnTrajectory(AUTO_NAME + ".1"),
//				new ShootNoteCommand(5.0, true),
//				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToFirstShootLocation,
//				new ShootNoteCommand(1.0, false),
				pathToSecondShootLocation,
//				new WaitUntilCommand(() -> Fintake.getInstance().isHoldingPiece()),
//				new ShootNoteCommand(1.0, true),
//				pathToThirdShootLocation,
//				new ShootCommand(0.75, true, true),
//				new InstantCommand(() -> Shooter.getInstance().setDoIdling(false)),
//				new InstantCommand(() -> Superstructure.getInstance().setWantsIntaking(false)),
//				new ParallelCommandGroup(
//						new SequentialCommandGroup(
//								new WaitCommand(0.5),
//								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
//						),
				pathToThirdShootLocation,
				pathToFourthShootLocation
//				),
//				new ShootNoteCommand(2.0, true),
//				new SetIntakeForcedOutCommand(false)
		);
	}
}
