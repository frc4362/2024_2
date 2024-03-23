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

public class TrespassAuto extends SequentialCommandGroup {
	private static final String AUTO_NAME = "Trespass";
	public TrespassAuto() {
		final var drive = Swerve.getInstance();
		final var pathToFirstShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".1", true);
		final var pathToSecondShootLocation = drive.getTrackTrajectoryCommand(AUTO_NAME + ".2", false);

		addCommands(
				new SetIntakeForcedOutCommand(false),
				pathToFirstShootLocation,
				new ShootNoteCommand(5.0, true),
				new WaitCommand(4.0),
				pathToSecondShootLocation,
				new SetIntakeForcedOutCommand(false)
		);
	}
}
