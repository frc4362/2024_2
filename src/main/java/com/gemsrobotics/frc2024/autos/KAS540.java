package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.commands.ApproachNoteCommand;
import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootNoteCommand;
import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class KAS540 extends SequentialCommandGroup {

	private static final String AUTO_NAME = "KAS 540";

	public KAS540() {
		final var drive = Swerve.getInstance();
		final var driveTo1stShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".1", true);
		final var driveTo2ndShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".2", false);
		final var driveToPreload = drive.getTrackTrajectoryCommand(AUTO_NAME + ".3", false);
		//.4 is a placeholder so the trajectory works
		final var driveTo3rdShot = drive.getTrackTrajectoryCommand(AUTO_NAME + ".5", false);
		final var driveToMiddle = drive.getTrackTrajectoryCommand(AUTO_NAME + ".6", false);


		addCommands(
				drive.resetOdometryOnTrajectory(AUTO_NAME + ".1"),
//				new SetIntakeForcedOutCommand(true),
				new ParallelDeadlineGroup(
						driveTo1stShot,
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new ConditionalCommand(new ShootNoteCommand(2.0, true), new WaitCommand(0.25), () -> Fintake.getInstance().isHoldingPiece()),
				new ParallelDeadlineGroup(
						driveTo2ndShot,
						new SequentialCommandGroup(
								new WaitCommand(0.25)
//								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new ConditionalCommand(new ShootNoteCommand(2.0, true), new WaitCommand(0.25), () -> Fintake.getInstance().isHoldingPiece()),
				new ParallelDeadlineGroup(
						new SequentialCommandGroup(
								driveToPreload,
								new ApproachNoteCommand(1),
								new WaitCommand(0.1), // bouncing lol
								driveTo3rdShot
						),
						new SequentialCommandGroup(
								new WaitCommand(0.25)
//								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				new SetIntakeForcedOutCommand(false),
				new ConditionalCommand(new ShootNoteCommand(2.0, true), new WaitCommand(0.25), () -> Fintake.getInstance().isHoldingPiece()),
				new SetWantedStateCommand(Superstructure.WantedState.IDLE),
				driveToMiddle
		);
	}
}
