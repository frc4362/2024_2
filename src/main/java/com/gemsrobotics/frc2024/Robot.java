// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics.frc2024;

import com.gemsrobotics.frc2024.autos.AmpSideAuto;
import com.gemsrobotics.frc2024.autos.SourceSideAuto;
import com.gemsrobotics.frc2024.autos.TestAuto;
import com.gemsrobotics.frc2024.subsystems.*;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Robot extends TimedRobot {
	private static final String NT_KEY = "robot";
	private OI m_oi;
	private Superstructure m_superstructure;
	private Swerve m_drive;
	private Shooter m_shooter;
	private Fintake m_fintake;
	private Arm m_arm;
	private Climber m_climber;
	private LEDManager m_leds;

	private StringPublisher m_translationPublisher;
	private DoublePublisher m_rotationPublisher;
	private DoubleSubscriber m_shooterLeftRPS, m_shooterRightRPS;
	private DoubleSubscriber m_feederDutyCycle;
	private NewTargetServer m_vision;

	private SendableChooser<Command> m_chooser;

	@Override
	public void robotInit() {
		m_superstructure = Superstructure.getInstance();
		m_superstructure.setStrictLocalizationEnabled(true);
		m_drive = Swerve.getInstance();
		m_shooter = Shooter.getInstance();
		m_fintake = Fintake.getInstance();
		m_vision = NewTargetServer.getInstance();
		m_arm = Arm.getInstance();
		m_climber = Climber.getInstance();
		m_leds = LEDManager.getInstance();

		m_oi = OI.getInstance();

		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
		m_translationPublisher = myTable.getStringTopic("wanted_translation").publish();
		m_rotationPublisher = myTable.getDoubleTopic("wanted_rotation").publish();

		m_shooterLeftRPS = myTable.getDoubleTopic("shooter_set_left_rps").subscribe(0.0);
		m_shooterRightRPS = myTable.getDoubleTopic("shooter_set_right_rps").subscribe(0.0);
		m_feederDutyCycle = myTable.getDoubleTopic("feeder_duty").subscribe(0.0);

		SmartDashboard.putNumber("shooter_set_rps", 85.0);
		SmartDashboard.putNumber("shooter_angle_degrees", 0.0);

		CommandScheduler.getInstance().registerSubsystem(
				m_arm,
				m_drive,
				m_vision,
				m_shooter,
				m_fintake,
				m_climber,
				m_leds,
				Bender.getInstance(),
				SimpleTargetServer.getInstance(),
				m_superstructure
		);

		SmartDashboard.putData("Swerve Drive", m_drive.getObserver());

		m_chooser = new SendableChooser<>();
		m_chooser.setDefaultOption("None", new WaitCommand(1.0));
		m_chooser.addOption("5 Piece", new AmpSideAuto());
		m_chooser.addOption("Source Auto", new SourceSideAuto());
		m_chooser.addOption("Test Auto", new TestAuto());
//		m_chooser.addOption("Quasi-Forwards Char", m_drive.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
//		m_chooser.addOption("Quasi-Backwards Char", m_drive.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));
//		m_chooser.addOption("Accel Forwards Char", m_drive.runDriveDynamTest(SysIdRoutine.Direction.kForward));
//		m_chooser.addOption("Accel Backwards Char", m_drive.runDriveDynamTest(SysIdRoutine.Direction.kReverse));
		SmartDashboard.putData("Auto Chooser", m_chooser);

		// heh heh
		m_drive.getDaqThread().setThreadPriority(99);

//		SignalLogger.enableAutoLogging(false);
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
	}

	@Override
	public void autonomousInit() {
		m_superstructure.setAlliance();
		m_chooser.getSelected().schedule();
	}

	@Override
	public void teleopInit() {
		m_superstructure.setAlliance();
		m_superstructure.setStrictLocalizationEnabled(false);
		m_superstructure.setWantsIntaking(false);
		m_superstructure.setFeedingAllowed(true);
		m_fintake.setForcedOut(false);
	}

	private boolean wantsClimb1 = false, wantsClimb2 = false;

	@Override
	public void teleopPeriodic() {
		m_translationPublisher.set(m_oi.getWantedSwerveTranslation().toString());
		m_rotationPublisher.set(m_oi.getWantedSwerveRotation());

		if (m_oi.getCopilot().getXButton()) {
			// subwoofer shot
			m_superstructure.setOverrideShotParams(Constants.getShotParameters(0));
		} else {
			m_superstructure.setOverrideParamsCleared();
		}

		m_superstructure.setStrictLocalizationEnabled(m_oi.getCopilot().getStartButton() && m_oi.getCopilot().getBackButton());

		if (m_oi.getPilot().getStartButtonPressed()) {
			m_drive.seedFieldRelative();
		}

		final var wantsAmp = m_oi.getPilot().getRightBumper();

		m_superstructure.setWantsIntaking(m_oi.getPilot().getLeftTriggerAxis() > 0.5);
		m_superstructure.setWantsAmpSpit(m_oi.getPilot().getRightTriggerAxis() > 0.5);

		if (wantsAmp) {
			m_drive.setOpenLoopFaceHeadingJoysticks(
					m_oi.getWantedSwerveTranslation(),
					Rotation2d.fromDegrees(-90));
		} else if (m_oi.getPilot().getRightTriggerAxis() > 0.5
					 && m_superstructure.getState() != Superstructure.SystemState.CLIMBING
					 && m_superstructure.getState() != Superstructure.SystemState.CLIMBING_2
					 && !m_oi.getCopilot().getXButton()
		) {
			m_drive.setAimingAtGoal(m_oi.getWantedSwerveTranslation());
		} else {
			m_drive.setOpenLoopJoysticks(
					m_oi.getWantedSwerveTranslation(),
					m_oi.getWantedSwerveRotation(),
					m_oi.getWantsEvasion());
		}

		Bender.getInstance().setDoSlam(m_oi.getCopilot().getLeftBumper());

		// toggles for climb
		if (m_oi.getPilot().getYButtonPressed()) {
			wantsClimb1 = !wantsClimb1;
		}

		if (m_oi.getPilot().getBButtonPressed()) {
			wantsClimb2 = !wantsClimb2;
		}

//		if () {
//			m_fintake.setIntakeReset();
//		}
//
//		m_fintake.setStallIntakeBack(m_oi.getCopilot().getYButton());

		if (wantsAmp) {
			m_superstructure.setWantedState(Superstructure.WantedState.AMPING);
		} else if (wantsClimb2) {
			m_superstructure.setWantedState(Superstructure.WantedState.CLIMBING_2);
			m_climber.setVoltsProtected(12 * m_oi.getCopilot().getLeftTriggerAxis());
		} else if (wantsClimb1) {
			m_superstructure.setWantedState(Superstructure.WantedState.CLIMBING);
			m_climber.setVolts(0.0);
		} else if (m_oi.getPilot().getRightTriggerAxis() > 0.5) {
			m_superstructure.setWantedState(Superstructure.WantedState.SHOOTING);
		} else if (m_oi.getPilot().getLeftTriggerAxis() > 0.5) {
			m_superstructure.setWantedState(Superstructure.WantedState.INTAKING);
		} else {
			m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
		}
	}
}
