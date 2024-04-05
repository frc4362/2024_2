package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.Shooter;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;

public class ShootNoteCommand extends SequentialCommandGroup {
	public ShootNoteCommand(final double timeout, final boolean stopShooting) {
		addCommands(
				new ParallelRaceGroup(
						new WaitCommand(timeout),
						new SequentialCommandGroup(
								new InstantCommand(() -> Superstructure.getInstance().setFeedingAllowed(false)),
								new SetWantedStateCommand(Superstructure.WantedState.SHOOTING),
								new ParallelDeadlineGroup(
										new SequentialCommandGroup(
												new WaitUntilCommand(() -> Swerve.getInstance().getAimingError().getDegrees() < 1.25),
												new InstantCommand(() -> Superstructure.getInstance().setFeedingAllowed(true)),
												new WaitUntilCommand(() -> Superstructure.getInstance().isReadyToShoot(true)),
												new WaitUntilCommand(() -> Shooter.getInstance().getMeasuredCurrentDraw() > 40.0),
												new WaitCommand(0.5)
										),
										new RunCommand(() -> Swerve.getInstance().setAimingAtGoal(new Translation2d()))
								)
						)
				).andThen(new SetWantedStateCommand(Superstructure.WantedState.IDLE).onlyIf(() -> stopShooting)),
				new InstantCommand(() -> Fintake.getInstance().clearPiece())
		);
	}
}
