# -*- coding: utf-8 -*-

"""Pinger model."""

import numpy as np
from scipy import signal as sp

__author__ = "Anass Al-Wohoush"


def generate_signal(buffersize, target_freq, time_offset, fs,
                    snr=20, pulse_length=1.3e-3, sweep=200):
    """Creates time shifted signal.

    Args:
        buffersize: Buffersize.
        target_freq: Target frequency in Hz.
        time_offset: Time offset in seconds.
        fs: Sampling frequency in Hz.
        snr: Signal-to-noise ratio in dB (default: 20 dB).
        pulse_length: Pulse length in seconds (default: 1.3 ms).
        sweep: Frequency sweep in Hz (default: 200 Hz).

    Returns:
        Signal.
    """
    # Setup random signal of the correct buffersize.
    signal = np.random.normal(0, 1, buffersize)

    # Generate signal as linear chirp to imperfect the signal.
    # The chirp spans the length of the ping and sweeps from f0 to f1 Hz
    # between t[0] to t1. A phi of 90 degrees is also added so that the signal
    # starts smoothly.
    ping = int(round(pulse_length * fs))
    time = np.arange(ping) / float(fs)
    chirp = 10 ** (snr / 20) * sp.chirp(
        t=time, t1=time[ping / 4],
        f0=max(0, target_freq - 200), f1=target_freq,
        phi=90
    )

    # Convert the time offset into units of sampling frequency.
    delta = np.ceil(time_offset * fs)

    # Add constant offset to allow for negative time offsets.
    constant_offset = int(buffersize / 2)

    # Add pulse to noise.
    for i in range(ping):
        try:
            signal[i + delta + constant_offset] += chirp[i]
        except IndexError:
            # Chirp is doesn't completely fit in the signal
            continue

    return signal
