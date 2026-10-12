import time

import numpy as np
import uhd

# --------------------------------------------------
# Configuration
# --------------------------------------------------

CENTER_FREQ = 5.740e9
SAMPLE_RATE = 2e6
RX_GAIN = 20

NUM_SAMPLES = 16384
CAPTURE_DELAY = 0.1
RX_TIMEOUT = 1.0


def analyze_channels(rx0, rx1):
    # Average received power (relative to full scale)
    power0 = np.mean(np.abs(rx0) ** 2)
    power1 = np.mean(np.abs(rx1) ** 2)

    p0_db = 10 * np.log10(power0 + 1e-15)
    p1_db = 10 * np.log10(power1 + 1e-15)

    # Positive difference means RX1 is stronger
    power_difference = p1_db - p0_db

    # Relative phase: angle(conj(RX0) * RX1)
    correlation = np.vdot(rx0, rx1)
    phase_deg = np.degrees(np.angle(correlation))

    # Normalized complex correlation magnitude
    # 1.0 = strongly correlated
    # 0.0 = uncorrelated
    coherence = np.abs(correlation) / (
        np.sqrt(np.sum(np.abs(rx0) ** 2) *
                np.sum(np.abs(rx1) ** 2)) + 1e-15
    )

    return p0_db, p1_db, power_difference, phase_deg, coherence


def main():
    usrp = None
    rx_streamer = None

    try:
        print("Connecting to B210...")

        usrp = uhd.usrp.MultiUSRP("")

        for ch in (0, 1):
            usrp.set_rx_rate(SAMPLE_RATE, ch)
            usrp.set_rx_freq(CENTER_FREQ, ch)
            usrp.set_rx_gain(RX_GAIN, ch)
            usrp.set_rx_antenna("RX2", ch)

        print("B210 connected.")
        print(f"Frequency: {CENTER_FREQ / 1e6:.1f} MHz")
        print(f"Sample rate: {SAMPLE_RATE / 1e6:.1f} MS/s")
        print(f"RX gain: {RX_GAIN} dB")

        stream_args = uhd.usrp.StreamArgs("fc32", "sc16")
        stream_args.channels = [0, 1]

        rx_streamer = usrp.get_rx_stream(stream_args)

        max_samps = rx_streamer.get_max_num_samps()

        recv_buffer = np.empty(
            (2, max_samps), dtype=np.complex64
        )

        samples = np.empty(
            (2, NUM_SAMPLES), dtype=np.complex64
        )

        metadata = uhd.types.RXMetadata()

        print("\nStarting receiver calibration...")
        print("Press Ctrl+C to stop.\n")

        while True:
            stream_cmd = uhd.types.StreamCMD(
                uhd.types.StreamMode.num_done
            )

            stream_cmd.num_samps = NUM_SAMPLES
            stream_cmd.stream_now = False

            stream_cmd.time_spec = (
                usrp.get_time_now()
                + uhd.types.TimeSpec(CAPTURE_DELAY)
            )

            rx_streamer.issue_stream_cmd(stream_cmd)

            total_received = 0

            while total_received < NUM_SAMPLES:
                remaining = NUM_SAMPLES - total_received
                request = min(remaining, max_samps)

                received = rx_streamer.recv(
                    recv_buffer[:, :request],
                    metadata,
                    timeout=RX_TIMEOUT
                )

                if metadata.error_code != (
                    uhd.types.RXMetadataErrorCode.none
                ):
                    raise RuntimeError(
                        f"RX error: {metadata.strerror()}"
                    )

                if received == 0:
                    raise RuntimeError("No samples received")

                samples[
                    :, total_received:total_received + received
                ] = recv_buffer[:, :received]

                total_received += received

            p0, p1, dp, phase, coherence = analyze_channels(
                samples[0], samples[1]
            )

            print(
                f"RX0: {p0:7.2f} dB | "
                f"RX1: {p1:7.2f} dB | "
                f"ΔP: {dp:+6.2f} dB | "
                f"Phase: {phase:+7.2f}° | "
                f"Corr: {coherence:.4f}"
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping reception...")

    finally:
        print("Cleaning up UHD resources...")

        if rx_streamer is not None:
            try:
                stop_cmd = uhd.types.StreamCMD(
                    uhd.types.StreamMode.stop_cont
                )
                rx_streamer.issue_stream_cmd(stop_cmd)
            except RuntimeError as error:
                print(f"Stream stop warning: {error}")

        rx_streamer = None
        usrp = None

        print("Cleanup completed.")


if __name__ == "__main__":
    main()