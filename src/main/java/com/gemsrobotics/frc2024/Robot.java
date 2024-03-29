// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics.frc2024;

import com.gemsrobotics.frc2024.autos.*;
import com.gemsrobotics.frc2024.subsystems.*;
import com.gemsrobotics.frc2024.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
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
//		m_chooser.addOption("Amp-Side Auto", new AmpSideAuto());
//		m_chooser.addOption("Source-Side Auto 2 First", new SourceSideAuto2First());
		m_chooser.addOption("Source-Side Auto 1 First", new SourceSideAuto1First());
//		m_chooser.addOption("Safe Auto", new SafeAuto());
//		m_chooser.addOption("Trespass Auto", new TrespassAuto());
//		m_chooser.addOption("Center Auto", new CenterAuto());
//		m_chooser.addOption("Shoot Note", new ShootNoteCommand(5.0, true));
//		m_chooser.addOption("Quasi-Forwards Char", m_drive.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
//		m_chooser.addOption("Quasi-Backwards Char", m_drive.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));
//		m_chooser.addOption("Accel Forwards Char", m_drive.runDriveDynamTest(SysIdRoutine.Direction.kForward));
//		m_chooser.addOption("Accel Backwards Char", m_drive.runDriveDynamTest(SysIdRoutine.Direction.kReverse));
		SmartDashboard.putData("Auto Chooser", m_chooser);

		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

		// heh heh
		m_drive.getDaqThread().setThreadPriority(99);

		// ehehe heheheheheh eehheehhehn
		RobotController.setBrownoutVoltage(5.0);
	}

	@Override
	public void disabledInit() {
		m_drive.setBraking();
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
		SmartDashboard.putBoolean("holding note", m_fintake.isHoldingPiece());
	}

	@Override
	public void autonomousInit() {
		m_superstructure.setAlliance();
		m_chooser.getSelected().schedule();
		m_superstructure.setStrictLocalizationEnabled(false);
	}

	@Override
	public void teleopInit() {
		m_superstructure.setAlliance();
		m_superstructure.setStrictLocalizationEnabled(false);
		m_superstructure.setWantsIntaking(false);
		m_superstructure.setFeedingAllowed(true);
		m_fintake.setForcedOut(false);

		// dont ask....
		// flush controls
		m_oi.getPilot().getYButtonPressed();
		m_oi.getPilot().getBButtonPressed();
		m_oi.getPilot().getStartButtonPressed();
		m_oi.getCopilot().getLeftBumperPressed();
		m_oi.getCopilot().getRightBumperPressed();
		m_oi.getCopilot().getYButtonReleased();
	}

	private boolean wantsClimb1 = false, wantsClimb2 = false;

	@Override
	public void teleopPeriodic() {
		m_translationPublisher.set(m_oi.getWantedSwerveTranslation().toString());
		m_rotationPublisher.set(m_oi.getWantedSwerveRotation());

		final boolean wantsPass;
		final var shotOverride = m_oi.getCopilotShotOverride();
		if (shotOverride.isEmpty()) {
			m_superstructure.setOverrideParamsCleared();
			wantsPass = false;
		} else {
			switch (shotOverride.get()) {
				case SUBWOOFER -> m_superstructure.setOverrideShotParams(Constants.getShotParameters(0.0));
				case FLAT -> m_superstructure.setOverrideShotParams(Constants.getFlatShot());
				default -> m_superstructure.setOverrideParamsCleared();
			}

			wantsPass = shotOverride.get() != OI.OverrideShotType.SUBWOOFER;
		}

		m_superstructure.setStrictLocalizationEnabled(m_oi.getCopilot().getStartButton() && m_oi.getCopilot().getBackButton());

		if (m_oi.getPilot().getStartButtonPressed()) {
			m_drive.seedFieldRelative();
		}

		final var wantsAmp = m_oi.getPilot().getRightBumper();
		final var wantsAmpSpit = m_oi.getPilot().getRightTriggerAxis() > 0.5;

		m_superstructure.setWantsIntaking(m_oi.getPilot().getLeftTriggerAxis() > 0.5);
		m_superstructure.setWantsAmpSpit(wantsAmpSpit);

		if (wantsAmp) {
			m_drive.setOpenLoopFaceHeadingJoysticks(
					m_oi.getWantedSwerveTranslation(),
					Rotation2d.fromDegrees(-90));
		} else if (wantsPass) {
			m_drive.setOpenLoopJoysticks(
					m_oi.getWantedSwerveTranslation(),
					m_oi.getWantedSwerveRotation(),
					m_oi.getWantsEvasion());
//			m_drive.setOpenLoopFaceHeadingJoysticks(
//					m_oi.getWantedSwerveTranslation(),
//					Constants.getAllianceConstants().getPassAngle());
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

		if (m_oi.getCopilot().getLeftBumperPressed()) {
			Constants.adjustShots(0.5);
		} else if (m_oi.getCopilot().getRightBumperPressed()) {
			Constants.adjustShots(-0.5);
		}

		Bender.getInstance().setDoSlam(m_oi.getCopilot().getAButton());
		Bender.getInstance().setDoRelease(m_oi.getCopilot().getBButton());

		// toggles for climb
		if (m_oi.getPilot().getYButtonPressed()) {
			wantsClimb1 = !wantsClimb1;
		}

		if (m_oi.getPilot().getBButtonPressed()) {
			wantsClimb2 = !wantsClimb2;
		}

		m_fintake.setStallIntakeBack(m_oi.getCopilot().getYButton());
		if (m_oi.getCopilot().getYButtonReleased()) {
			m_fintake.setIntakeReset();
		}

		m_superstructure.setFeedingAllowed(m_oi.getPilot().getRightTriggerAxis() > 0.5);
		if (wantsAmp && wantsAmpSpit) {
			m_superstructure.setWantedState(Superstructure.WantedState.AMPING);
		} else if (wantsPass) {
			m_superstructure.setWantedState(Superstructure.WantedState.PASSING);
		} else if (wantsClimb2) {
			m_superstructure.setWantedState(Superstructure.WantedState.CLIMBING_2);
			m_climber.setVoltsProtected(10 * m_oi.getCopilot().getLeftTriggerAxis());
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
