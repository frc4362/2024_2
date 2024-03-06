package com.gemsrobotics.lib;

import java.util.function.Function;

public class TimestampedValue<T> {
	private final double m_timestampSeconds;
	private final T m_value;

	public TimestampedValue(final double timeSeconds, final T value) {
		m_timestampSeconds = timeSeconds;
		m_value = value;
	}

	public double getTimestamp() {
		return m_timestampSeconds;
	}

	public T getValue() {
		return m_value;
	}

	public <H> TimestampedValue<H> map(final Function<T, H> f) {
		return new TimestampedValue<>(m_timestampSeconds, f.apply(m_value));
	}
}
