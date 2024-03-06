package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootCommand;
import com.gemsrobotics.frc2024.subsystems.Shooter;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.*;

public class FivePieceAuto extends SequentialCommandGroup {
	public FivePieceAuto() {
		final var drive = Swerve.getInstance();
		final var pathToFirstShootLocation = drive.getTrackTrajectoryCommand("Milford5.1", true);
		final var pathToSecondShootLocation = drive.getTrackTrajectoryCommand("Milford5.2", false);
		final var pathToThirdShootLocation = drive.getTrackTrajectoryCommand("Milford5.3", false);
		final var pathToFourthShootLocation = drive.getTrackTrajectoryCommand("Milford5.4", false);

		addCommands(
				pathToFirstShootLocation,
				new ShootCommand(1.0, true, true),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				new SetIntakeForcedOutCommand(true),
				new WaitCommand(0.1),
				pathToSecondShootLocation,
				new WaitCommand(0.1),
				new ShootCommand(1.0, true, true),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToThirdShootLocation,
				new ShootCommand(1.0, true, true),
				new WaitCommand(0.1),
				new SetWantedStateCommand(Superstructure.WantedState.INTAKING),
				pathToFourthShootLocation,
				new ShootCommand(1.0, true, true),
				new SetIntakeForcedOutCommand(false)
		);
	}
}
