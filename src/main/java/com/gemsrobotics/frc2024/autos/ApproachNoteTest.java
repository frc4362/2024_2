package com.gemsrobotics.frc2024.autos;

import com.gemsrobotics.frc2024.commands.ApproachNoteCommand;
import com.gemsrobotics.frc2024.commands.SetIntakeForcedOutCommand;
import com.gemsrobotics.frc2024.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2024.commands.ShootNoteCommand;
import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.*;

public class ApproachNoteTest extends SequentialCommandGroup {

	//private static final String AUTO_NAME = "";

	public ApproachNoteTest() {
		//final var drive = Swerve.getInstance();
		addCommands(
				//drive.resetOdometryOnTrajectory(AUTO_NAME + ".1"),
				//new SetIntakeForcedOutCommand(true),
				new ParallelDeadlineGroup(
						new ApproachNoteCommand(3),
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new SetWantedStateCommand(Superstructure.WantedState.INTAKING)
						)
				),
				//new SetIntakeForcedOutCommand(false),
				new SetWantedStateCommand(Superstructure.WantedState.IDLE)
		);
	}
}
