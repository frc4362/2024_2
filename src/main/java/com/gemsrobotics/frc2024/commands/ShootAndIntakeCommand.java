package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootAndIntakeCommand extends SequentialCommandGroup {
	public ShootAndIntakeCommand(final double duration, final boolean stopShooting) {
		addCommands(
				new InstantCommand(() -> Superstructure.getInstance().setWantsIntaking(true)),
				new SetWantedStateCommand(Superstructure.WantedState.SHOOTING),
				new WaitCommand(duration),
				new SetWantedStateCommand(Superstructure.WantedState.IDLE).onlyIf(() -> stopShooting),
				new InstantCommand(() -> Superstructure.getInstance().setWantsIntaking(false)).onlyIf(() -> stopShooting)
		);
	}
}
