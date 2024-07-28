package com.gemsrobotics.frc2024.commands;

import com.gemsrobotics.frc2024.subsystems.Fintake;
import com.gemsrobotics.frc2024.subsystems.NoteDetector;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.*;

public class ApproachNoteCommand extends SequentialCommandGroup {
    public ApproachNoteCommand(final double timeout) {
        addCommands(
                new ParallelRaceGroup(
                        new WaitCommand(timeout),
                        new RepeatCommand(
                                new InstantCommand(() -> Swerve.getInstance().approachNote())
                        ).until(() -> NoteDetector.getInstance().getVehicleToNoteRotation().isEmpty() || Fintake.getInstance().isHoldingPiece())
                        //maybe remove the NoteDetector portion if it causes the command to exit early due to a single frame of missed detection
                ),
                new InstantCommand(() -> Swerve.getInstance().setBraking())
        );
    }
}
