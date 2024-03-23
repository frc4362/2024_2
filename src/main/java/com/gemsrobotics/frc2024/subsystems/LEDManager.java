package com.gemsrobotics.frc2024.subsystems;

import java.util.Objects;

import com.ctre.phoenix.led.*;
import com.gemsrobotics.frc2024.Constants;

import com.gemsrobotics.frc2024.OI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class LEDManager implements Subsystem {
    private static LEDManager INSTANCE = null;
	public static LEDManager getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new LEDManager();
		}

		return INSTANCE;
	}

	public static class PeriodicIO {
		public LEDstate state = LEDstate.IDLE;
	}


    public enum LEDstate {
		IDLE,
		NOTE_GET,
        OUT_OF_RANGE,
		IN_RANGE,
		BAD_TAGS,
		BAD
    }

	private final StringPublisher m_statePublisher;
	private final CANdle m_candle;
	private final PeriodicIO m_periodicIO;
	private final Timer m_noteGetTimer;

    private LEDManager() {
        m_candle = new CANdle(59, Constants.AUX_BUS_NAME);

        CANdleConfiguration configAll = new CANdleConfiguration();
		configAll.statusLedOffWhenActive = true;
		configAll.disableWhenLOS = false;
		configAll.stripType = CANdle.LEDStripType.RGB;
		configAll.brightnessScalar = 1.0;
		configAll.vBatOutputMode = CANdle.VBatOutputMode.Off;
		m_candle.configAllSettings(configAll, 100);
		m_candle.clearAnimation(0);
		m_candle.setLEDs(0, 0, 0);

		m_noteGetTimer = new Timer();
		m_noteGetTimer.stop();

		m_periodicIO = new PeriodicIO();

		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable("leds");
		m_statePublisher = myTable.getStringTopic("state").publish();
    }

	@Override
	public void periodic() {
		if (m_noteGetTimer.hasElapsed(1.0)) {
			m_noteGetTimer.stop();
			m_noteGetTimer.reset();
			m_candle.clearAnimation(0);
		} else if (m_noteGetTimer.get() > 0) {
			m_statePublisher.set("blocked by note pickup");
			return;
		}

		OI m_oi = OI.getInstance();
		m_oi.getPilot().setRumble(GenericHID.RumbleType.kBothRumble, 0.0); // 0 to 1
		m_oi.getCopilot().setRumble(GenericHID.RumbleType.kBothRumble, 0.0); // 0 to 1

		m_statePublisher.set(m_periodicIO.state.toString());
		switch (m_periodicIO.state) {
			case IDLE:
//				m_candle.setLEDs(15,15,15);
				m_candle.setLEDs(0,0,0);
				break;
			case NOTE_GET:
				m_candle.setLEDs(0,255,0);
				break;
			case OUT_OF_RANGE:
				m_candle.setLEDs(255 / 3,255 / 3,0);
				break;
			case IN_RANGE:
				m_candle.setLEDs(0, 255 / 5, 125 / 5);
				break;
			case BAD_TAGS:
				m_candle.setLEDs(0x8f, 0x00, 0xFF);
				break;
			case BAD:
				m_candle.setLEDs(255,0,0);
				break;
			default:
				m_candle.setLEDs(0,0,0);
		}
	}

    public void setLEDS(final LEDstate state) {
		m_periodicIO.state = state;
    }

	private static final Animation NOTE_GET_ANIMATION = new RainbowAnimation();
	public void playNoteGetAnimation() {
		m_noteGetTimer.start();
		m_candle.animate(NOTE_GET_ANIMATION);
		OI m_oi = OI.getInstance();
		m_oi.getPilot().setRumble(GenericHID.RumbleType.kBothRumble, 0.55); // 0 to 1
		m_oi.getCopilot().setRumble(GenericHID.RumbleType.kBothRumble, 0.55); // 0 to 1
	}
}
