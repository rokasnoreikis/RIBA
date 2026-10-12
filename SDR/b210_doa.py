
import time

import numpy as np
import uhd

# ============================================================
# Configuration
# ============================================================

CENTER_FREQ = 5.740e9
SAMPLE_RATE = 2e6
RX_GAIN = 30
ANTENNA_SPACING = 0.028

C = 299792458
WAVELENGTH = C / CENTER_FREQ

NUM_SAMPLES = 16384
CAPTURE_DELAY = 0.1
RX_TIMEOUT = 1.0

# Zero-degree calibration (not yet measured)
PHASE_CAL_DEG = -9.0


def calculate_doa(rx0, rx1):
    """Estimate relative phase and uncalibrated DOA."""

    # Digital received power (dB relative to full-scale amplitude)
    p0 = 10 * np.log10(
        np.mean(np.abs(rx0) ** 2) + 1e-15
    )

    p1 = 10 * np.log10(
        np.mean(np.abs(rx1) ** 2) + 1e-15
    )

    # Cross-correlation between channels
    correlation = np.vdot(rx0, rx1)

    # Relative phase
    phase_raw = np.angle(correlation)

    # Remove calibration phase and wrap to [-pi, pi]
    phase = np.angle(
        np.exp(
            1j * (
                phase_raw - np.deg2rad(PHASE_CAL_DEG)
            )
        )
    )

    # Estimate DOA
    argument = (
        phase * WAVELENGTH /
        (2 * np.pi * ANTENNA_SPACING)
    )

    argument = np.clip(argument, -1.0, 1.0)

    theta = np.degrees(np.arcsin(argument))

    return p0, p1, np.degrees(phase_raw), theta


def main():
    usrp = None
    rx_streamer = None

    try:
        # ====================================================
        # Initialize B210
        # ====================================================

        print("Connecting to B210...")

        usrp = uhd.usrp.MultiUSRP("")

        for ch in (0, 1):
            usrp.set_rx_rate(SAMPLE_RATE, ch)
            usrp.set_rx_freq(CENTER_FREQ, ch)
            usrp.set_rx_gain(RX_GAIN, ch)
            usrp.set_rx_antenna("RX2", ch)

        # ====================================================
        # Configure dual-channel RX
        # ====================================================

        stream_args = uhd.usrp.StreamArgs(
            "fc32", "sc16"
        )
        stream_args.channels = [0, 1]

        rx_streamer = usrp.get_rx_stream(stream_args)

        # UHD receives into this contiguous buffer.
        max_samps = rx_streamer.get_max_num_samps()

        recv_buffer = np.empty(
            (2, max_samps),
            dtype=np.complex64
        )

        samples = np.empty(
            (2, NUM_SAMPLES),
            dtype=np.complex64
        )

        metadata = uhd.types.RXMetadata()

        print("B210 connected.")
        print(f"Frequency: {CENTER_FREQ / 1e6:.1f} MHz")
        print(f"Sample rate: {SAMPLE_RATE / 1e6:.1f} MS/s")
        print(f"Antenna spacing: {ANTENNA_SPACING * 1000:.1f} mm")
        print("\nStarting DOA measurements...")
        print("Press Ctrl+C to stop.\n")

        # ====================================================
        # Main acquisition loop
        # ====================================================

        while True:

            stream_cmd = uhd.types.StreamCMD(
                uhd.types.StreamMode.num_done
            )

            stream_cmd.num_samps = NUM_SAMPLES
            stream_cmd.stream_now = False

            # Both channels start at the same device time
            stream_cmd.time_spec = (
                usrp.get_time_now()
                + uhd.types.TimeSpec(CAPTURE_DELAY)
            )

            rx_streamer.issue_stream_cmd(stream_cmd)

            total_received = 0

            # Collect the complete capture
            while total_received < NUM_SAMPLES:

                remaining = NUM_SAMPLES - total_received
                request = min(remaining, max_samps)

                received = rx_streamer.recv(
                    recv_buffer[:, :request],
                    metadata,
                    timeout=RX_TIMEOUT,
                )

                if (
                    metadata.error_code
                    != uhd.types.RXMetadataErrorCode.none
                ):
                    raise RuntimeError(
                        f"RX error: {metadata.strerror()}"
                    )

                if received == 0:
                    raise RuntimeError(
                        "No samples received from B210"
                    )

                samples[
                    :, total_received:total_received + received
                ] = recv_buffer[:, :received]

                total_received += received

            # ================================================
            # DOA processing
            # ================================================

            rx0 = samples[0]
            rx1 = samples[1]

            p0, p1, phase_deg, theta = calculate_doa(
                rx0, rx1
            )

            print(
                f"RX0: {p0:7.2f} dB | "
                f"RX1: {p1:7.2f} dB | "
                f"Phase: {phase_deg:7.2f} deg | "
                f"DOA: {theta:7.2f} deg"
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping reception...")

    finally:
        # ====================================================
        # Cleanup
        # ====================================================

        print("Cleaning up UHD resources...")

        if rx_streamer is not None:
            try:
                stop_cmd = uhd.types.StreamCMD(
                    uhd.types.StreamMode.stop_cont
                )

                rx_streamer.issue_stream_cmd(stop_cmd)

            except RuntimeError as error:
                print(f"Stream stop warning: {error}")

        # Release UHD objects
        rx_streamer = None
        usrp = None

        print("Cleanup completed.")


if __name__ == "__main__":
    main()
