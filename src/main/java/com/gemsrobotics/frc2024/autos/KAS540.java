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
		final var driveToPickup = drive.getTrackTrajectoryCommand(AUTO_NAME + " 1" + ".1", true);
		final var driveTo1stShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".1", true);
		final var driveTo2ndShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".2", false);
		final var driveToPreload = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".3", false);
		// .4 is fake and not real to account for movement from picking up the note with the ApproachNoteCommand
		final var driveTo3rdShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".5", false);
		final var driveToMiddle = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".6", false);

		//final var driveToPickup = drive.getTrackTrajectoryCommand(AUTO_NAME + " 1" + ".1", true);
		//final var driveTo1stShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".1", false);
		//final var driveTo2ndShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".2", false);
		//final var driveTo3rdShot = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".3", false);
		//final var driveToMiddle = drive.getTrackTrajectoryCommand(AUTO_NAME + " 2" + ".4", false);


		addCommands(
				drive.resetOdometryOnTrajectory(AUTO_NAME + " 1" + ".1"),
//				new SetIntakeForcedOutCommand(true),
				new ParallelDeadlineGroup(
						new SequentialCommandGroup(
								driveToPickup,
								new WaitCommand(0.1), // bouncing lol
								driveTo1stShot
						),
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
								driveToPreload, // drive most of the way to est. preload location
								new ApproachNoteCommand(1.5), // use vision to find the note once nearby
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
