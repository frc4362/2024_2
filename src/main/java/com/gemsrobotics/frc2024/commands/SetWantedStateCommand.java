package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetWantedStateCommand extends InstantCommand {
	public SetWantedStateCommand(final Superstructure.WantedState wantedState) {
		super(() -> Superstructure.getInstance().setWantedState(wantedState));
	}
}
