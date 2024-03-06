package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.Constants;
import com.gemsrobotics.frc2024.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SpeakerShotCommand extends SequentialCommandGroup {
	public SpeakerShotCommand(final double duration) {
		addCommands(
				new InstantCommand(() -> Superstructure.getInstance().setOverrideShotParams(Constants.getShotParameters(0.0))),
				new SetWantedStateCommand(Superstructure.WantedState.SHOOTING),
				new WaitCommand(duration),
				new SetWantedStateCommand(Superstructure.WantedState.IDLE),
				new InstantCommand(() -> Superstructure.getInstance().setOverrideParamsCleared())
		);
	}
}
