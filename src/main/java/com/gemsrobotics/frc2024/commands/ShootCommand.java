package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;

public class ShootCommand extends SequentialCommandGroup {
	public ShootCommand(final double duration, final boolean stopShooting, final boolean doAutoAim) {
		addCommands(
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								new InstantCommand(() -> Superstructure.getInstance().setFeedingAllowed(!doAutoAim)),
								new SetWantedStateCommand(Superstructure.WantedState.SHOOTING),
								new InstantCommand(() -> Superstructure.getInstance().setFeedingAllowed(true)),
								new WaitCommand(duration)
						),
						new RunCommand(() -> Swerve.getInstance().setAimingAtGoal(new Translation2d()))
								.until(() -> Swerve.getInstance().getAimingError().getDegrees() < 1.5).onlyIf(() -> doAutoAim)
				).withTimeout(duration).andThen(new SetWantedStateCommand(Superstructure.WantedState.IDLE).onlyIf(() -> stopShooting))
		);
	}
}
