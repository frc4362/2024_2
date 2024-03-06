package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.subsystems.Fintake;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIntakeForcedOutCommand extends InstantCommand {
	public SetIntakeForcedOutCommand(final boolean forcedOut) {
		super(() -> {
			Fintake.getInstance().setForcedOut(forcedOut);
		});
	}
}
