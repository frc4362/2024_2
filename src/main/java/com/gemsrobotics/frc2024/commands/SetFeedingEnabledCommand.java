package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetFeedingEnabledCommand extends InstantCommand {
	public SetFeedingEnabledCommand(final boolean feedingEnabled) {
		super(() -> {
			Superstructure.getInstance().setFeedingAllowed(feedingEnabled);
		});
	}
}
