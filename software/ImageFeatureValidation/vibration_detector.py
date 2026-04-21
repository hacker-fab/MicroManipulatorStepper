"""
Vibration Detection Tool for Basler Camera

Provides commands for:
1. Recording high-framerate video from a Basler camera
2. Analyzing video frames to detect vibration frequencies
"""

import cv2
import numpy as np
from pathlib import Path
from typer import Typer
from datetime import datetime
import time
from scipy import signal
from scipy import fft as scipy_fft
import csv
from scipy import signal
from scipy.fft import fft, fftfreq


from pypylon import pylon


APP = Typer()


class BaslerCamera:
    """
    High-performance Basler camera interface using full pylon pipeline.
    Designed for maximum throughput and minimal dropped frames.
    """

    def __init__(self, camera_index=0):
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()

        if camera_index >= len(devices):
            raise RuntimeError(f"No camera at index {camera_index} (found {len(devices)})")

        self.camera = pylon.InstantCamera(
            tl_factory.CreateDevice(devices[camera_index])
        )

        self.converter = None

    def open(
        self,
        target_fps=None,
        max_fps=True,
        exposure_us=None,
        pixel_format="Mono8",   # "Mono8" or "RGB8"
        buffer_count=128,
        width=None,
        height=None,
    ):
        self.camera.Open()

        print(f"Using: {self.camera.GetDeviceInfo().GetModelName()}")

        # -------------------------
        # Reduce resolution for higher FPS
        # -------------------------
        if width is not None:
            try:
                # Set width - align to camera increment
                inc = self.camera.Width.Inc
                width = (width // inc) * inc
                if width >= self.camera.Width.Min and width <= self.camera.Width.Max:
                    self.camera.Width.SetValue(width)
                    print(f"Width set to: {self.camera.Width.GetValue()}")
            except Exception as e:
                print(f"Warning: Could not set width: {e}")
        
        if height is not None:
            try:
                # Set height - align to camera increment
                inc = self.camera.Height.Inc
                height = (height // inc) * inc
                if height >= self.camera.Height.Min and height <= self.camera.Height.Max:
                    self.camera.Height.SetValue(height)
                    print(f"Height set to: {self.camera.Height.GetValue()}")
            except Exception as e:
                print(f"Warning: Could not set height: {e}")

        # -------------------------
        # Pixel format (CRITICAL for bandwidth)
        # -------------------------
        if pixel_format == "Mono8":
            self.camera.PixelFormat.SetValue("Mono8")
        else:
            self.camera.PixelFormat.SetValue("RGB8")
        
        print(f"Pixel format: {self.camera.PixelFormat.GetValue()}")

        # -------------------------
        # Exposure (lower = faster)
        # -------------------------
        if exposure_us is None:
            self.camera.ExposureTime.SetValue(self.camera.ExposureTime.Min)
        else:
            self.camera.ExposureTime.SetValue(exposure_us)

        print(f"Exposure: {self.camera.ExposureTime.GetValue():.1f} us")

        # -------------------------
        # FPS Control
        # -------------------------
        try:
            if target_fps is not None:
                # Enable framerate control and set target
                self.camera.AcquisitionFrameRateEnable.SetValue(True)
                self.camera.AcquisitionFrameRate.SetValue(target_fps)
                actual_fps = self.camera.AcquisitionFrameRate.GetValue()
                print(f"FPS control enabled: {actual_fps:.1f} FPS target")
            else:
                # Disable frame rate limiting for max speed
                self.camera.AcquisitionFrameRateEnable.SetValue(False)
                print(f"FPS control disabled (max speed mode)")
        except Exception as e:
            print(f"Warning: Could not set framerate control: {e}")

        # -------------------------
        # Maximize throughput with larger buffers
        # -------------------------
        self.camera.MaxNumBuffer = buffer_count
        self.camera.OutputQueueSize = buffer_count
        print(f"Buffer count: {buffer_count}")

        # -------------------------
        # Optimize USB/GigE performance (pylon SDK documentation)
        # -------------------------
        try:
            # Maximize USB transfer size for faster data throughput
            # This is critical for high-framerate operation
            self.camera.StreamGrabber.MaxTransferSize.SetValue(4194304)  # 4 MB blocks
            print(f"MaxTransferSize: {self.camera.StreamGrabber.MaxTransferSize.GetValue() / 1024 / 1024:.1f} MB")
        except:
            pass  # Feature may not be available on all interfaces

        try:
            # Increase number of buffers in stream grabber for better pipelining
            self.camera.StreamGrabber.NumBuffers.SetValue(buffer_count)
        except:
            pass

        try:
            # Request buffer timeout - shorter is better for throughput
            self.camera.StreamGrabber.GrabCooldown.SetValue(0)
        except:
            pass

        # -------------------------
        # Converter (only if needed)
        # -------------------------
        self.converter = pylon.ImageFormatConverter()

        if pixel_format == "Mono8":
            self.converter.OutputPixelFormat = pylon.PixelType_Mono8
        else:
            self.converter.OutputPixelFormat = pylon.PixelType_RGB8packed

        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def start(self):
        # LatestImageOnly allows pipelined grabbing - camera fills buffers while we retrieve
        # This eliminates serialization bottleneck at the cost of dropping some frames
        # For vibration analysis, continuous fast frames > perfect frame delivery
        self.camera.StartGrabbing(
            pylon.GrabStrategy_LatestImageOnly,
            pylon.GrabLoop_ProvidedByUser,
        )

    def stop(self):
        if self.camera.IsGrabbing():
            self.camera.StopGrabbing()

    def close(self):
        self.stop()
        if self.camera.IsOpen():
            self.camera.Close()

    def grab(self, timeout=100):
        if not self.camera.IsGrabbing():
            return None

        try:
            grab = self.camera.RetrieveResult(timeout, pylon.TimeoutHandling_ThrowException)
        except pylon.TimeoutException:
            return None  # Frame dropped due to LatestImageOnly strategy

        if not grab.GrabSucceeded():
            grab.Release()
            return None

        img = self.converter.Convert(grab)
        arr = img.GetArray()
        grab.Release()
        return arr

    def get_dims(self):
        return int(self.camera.Width.Value), int(self.camera.Height.Value)

    def get_fps(self):
        return self.camera.AcquisitionFrameRate.GetValue()


@APP.command()
def record_video(
    output_path: Path = Path("vibration_video.avi"),
    duration_s: float = 10.0,
    framerate: float = 40.0,
    max_framerate: bool = False,
    exposure_us: float = 8333.0,
    pixel_format: str = "Mono8",
    width: int = None,
    height: int = None,
):
    """
    Record high-speed video from Basler camera.
    
    Args:
        output_path: Output video file path (default: vibration_video.avi)
        duration_s: Recording duration in seconds (default: 10.0)
        framerate: Target framerate (default: camera max)
        max_framerate: Use camera max framerate if True (default: True)
        pixel_format: "Mono8" or "RGB8" (default: Mono8 for better FPS)
        width: Video width in pixels (optional, default: camera max)
        height: Video height in pixels (optional, default: camera max)
    
    Example:
        # Record 5 seconds at max FPS with half resolution for faster grabbing
        record-video --duration-s 5 --width 960 --height 600
        
        # Record 30 seconds at fixed 100 FPS
        record-video --duration-s 30 --framerate 100 --max-framerate false
    """
    cam = BaslerCamera()

    cam.open(
        target_fps=framerate,
        max_fps=max_framerate,
        pixel_format=pixel_format,
        buffer_count=256,   # large buffer for burst stability
        width=width,
        height=height,
        exposure_us=exposure_us,
    )

    vid_width, vid_height = cam.get_dims()
    fps = cam.get_fps()

    print(f"Recording: {vid_width}x{vid_height} @ {fps:.1f} FPS, {pixel_format}")

    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    is_color = pixel_format != "Mono8"

    out = cv2.VideoWriter(
        str(output_path),
        fourcc,
        fps,
        (vid_width, vid_height),
        isColor=is_color,
    )

    if not out.isOpened():
        raise RuntimeError("Failed to open video writer")

    cam.start()

    frame_count = 0
    start = time.time()

    try:
        while True:
            frame = cam.grab()

            if frame is None:
                continue

            # Convert ONLY if needed
            if pixel_format == "Mono8":
                out.write(frame)
            else:
                out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

            frame_count += 1

            if (time.time() - start) >= duration_s:
                break

    finally:
        cam.stop()
        out.release()
        cam.close()

    elapsed = time.time() - start
    actual_fps = frame_count / elapsed

    print(f"\nRecorded {frame_count} frames in {elapsed:.2f}s")
    print(f"Actual framerate achieved: {actual_fps:.1f} FPS")

# Number of frames used to calibrate the dominant vibration axis.
CALIBRATION_FRAMES = 30
 
# Fraction of Nyquist above which we warn about possible aliasing.
ALIASING_THRESHOLD = 0.8
 
 
def _dominant_axis(flow_samples: list[np.ndarray]) -> str:
    """Return 'x' or 'y' — whichever axis has higher variance across samples."""
    var_x = np.var([np.mean(f[..., 0]) for f in flow_samples])
    var_y = np.var([np.mean(f[..., 1]) for f in flow_samples])
    return "x" if var_x >= var_y else "y"
 
 
def _signed_displacement(flow: np.ndarray, axis: str) -> float:
    """Return the mean signed displacement along the chosen axis."""
    channel = 0 if axis == "x" else 1
    return float(np.mean(flow[..., channel]))
 
 
def _crop(frame_gray: np.ndarray, roi: tuple[int, int, int, int] | None) -> np.ndarray:
    if roi is None:
        return frame_gray
    x, y, w, h = roi
    return frame_gray[y : y + h, x : x + w]
 
 
def _optical_flow(prev: np.ndarray, curr: np.ndarray) -> np.ndarray:
    return cv2.calcOpticalFlowFarneback(
        prev,
        curr,
        None,
        pyr_scale=0.5,
        levels=3,
        winsize=15,
        iterations=3,
        poly_n=5,
        poly_sigma=1.2,
        flags=0,
    )
 
 
@APP.command()
def analyze_vibrations(
    video_path: Path,
    output_csv: Path = None,
    fps_override: float | None = None,
    top_n: int = 10,
    roi_x: int = None,
    roi_y: int = None,
    roi_width: int = None,
    roi_height: int = None,
):
    """
    Analyze a video to detect vibration frequencies.
 
    Uses optical flow to track signed displacement along the dominant
    vibration axis, then extracts dominant frequencies via FFT.
 
    Args:
        video_path: Path to the video file.
        output_csv: Optional path to save results as CSV.
        fps_override: Override the FPS reported by the video container.
        top_n: Number of top frequencies to report (default: 10).
        roi_x: X coordinate of the top-left corner of the ROI.
        roi_y: Y coordinate of the top-left corner of the ROI.
        roi_width: Width of the ROI.
        roi_height: Height of the ROI.
    """
    video_path = Path(video_path)
    if not video_path.exists():
        print(f"Error: Video file not found: {video_path}")
        raise typer.Exit(1)
 
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print("Error: Could not open video file.")
        raise typer.Exit(1)
 
    container_fps = cap.get(cv2.CAP_PROP_FPS)
    fps = fps_override if fps_override is not None else container_fps
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    nyquist = fps / 2.0
 
    print(f"\nOpening video : {video_path}")
    print(f"  Resolution  : {width}x{height}")
    print(f"  Container FPS: {container_fps:.3f}")
    print(f"  Analysis FPS : {fps:.3f}  (Nyquist = {nyquist:.1f} Hz)")
    print(f"  Total frames : {frame_count}  ({frame_count / fps:.2f} s)")
 
    roi: tuple[int, int, int, int] | None = None
    if all(v is not None for v in (roi_x, roi_y, roi_width, roi_height)):
        roi = (roi_x, roi_y, roi_width, roi_height)
        print(f"  ROI         : x={roi_x} y={roi_y} w={roi_width} h={roi_height}")
    else:
        print("  ROI         : full frame")
 
    # ------------------------------------------------------------------ #
    # Phase 1 – calibration: determine dominant vibration axis            #
    # ------------------------------------------------------------------ #
    print(f"\nCalibrating axis on first {CALIBRATION_FRAMES} frames …")
 
    ret, prev_frame = cap.read()
    if not ret:
        print("Error: Could not read first frame.")
        raise typer.Exit(1)
 
    prev_gray = _crop(cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY), roi)
    calibration_flows: list[np.ndarray] = []
 
    for _ in range(CALIBRATION_FRAMES):
        ret, frame = cap.read()
        if not ret:
            break
        gray = _crop(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), roi)
        flow = _optical_flow(prev_gray, gray)
        calibration_flows.append(flow)
        prev_gray = gray
 
    if len(calibration_flows) < 5:
        print("Error: Not enough frames for calibration.")
        raise typer.Exit(1)
 
    axis = _dominant_axis(calibration_flows)
    print(f"  Dominant vibration axis: {axis.upper()}")
 
    # ------------------------------------------------------------------ #
    # Phase 2 – full pass: collect signed displacements                   #
    # ------------------------------------------------------------------ #
    print("\nAnalyzing motion …")
 
    # Seed the signal with the calibration frames we already computed.
    displacements: list[float] = [_signed_displacement(f, axis) for f in calibration_flows]
 
    frame_idx = CALIBRATION_FRAMES + 1  # already consumed that many frames
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
 
            gray = _crop(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), roi)
            flow = _optical_flow(prev_gray, gray)
            displacements.append(_signed_displacement(flow, axis))
            prev_gray = gray
            frame_idx += 1
 
            if frame_idx % max(1, frame_count // 20) == 0:
                pct = frame_idx / frame_count * 100
                print(f"  {frame_idx}/{frame_count} frames ({pct:.0f}%)", end="\r")
    except Exception as exc:
        print(f"\nError during analysis: {exc}")
        raise typer.Exit(1)
    finally:
        cap.release()
 
    n_frames = len(displacements)
    print(f"\n  Processed {n_frames} frames total.")
 
    # ------------------------------------------------------------------ #
    # Phase 3 – signal conditioning                                        #
    # ------------------------------------------------------------------ #
    sig = np.array(displacements, dtype=float)
 
    # Remove slow drift / DC offset via linear detrend.
    sig = signal.detrend(sig, type="linear")
 
    # Normalise to unit variance.
    std = np.std(sig)
    if std > 0:
        sig /= std
 
    # Hann window to suppress spectral leakage.
    window = signal.windows.hann(len(sig))
    sig_windowed = sig * window
 
    # ------------------------------------------------------------------ #
    # Phase 4 – FFT                                                        #
    # ------------------------------------------------------------------ #
    print("Computing FFT …")
    spectrum = np.abs(fft(sig_windowed))
    freqs = fftfreq(len(sig_windowed), d=1.0 / fps)
 
    # Positive frequencies only.
    pos = freqs > 0
    freqs_pos = freqs[pos]
    spectrum_pos = spectrum[pos]
 
    # Normalise spectrum to [0, 1].
    peak = np.max(spectrum_pos)
    if peak > 0:
        spectrum_pos = spectrum_pos / peak
 
    # ------------------------------------------------------------------ #
    # Phase 5 – report                                                     #
    # ------------------------------------------------------------------ #
    top_idx = np.argsort(spectrum_pos)[-top_n:][::-1]
    top_freqs = freqs_pos[top_idx]
    top_mags = spectrum_pos[top_idx]
 
    col = 72
    print(f"\n{'=' * col}")
    print(f"  TOP {top_n} VIBRATION FREQUENCIES  (Nyquist = {nyquist:.1f} Hz)")
    print(f"{'=' * col}")
    print(f"{'Rank':<6} {'Freq (Hz)':<14} {'Magnitude':<14} {'Period (ms)':<14} Notes")
    print(f"{'-' * col}")
 
    results = []
    for rank, (freq, mag) in enumerate(zip(top_freqs, top_mags), 1):
        period_ms = 1000.0 / freq if freq > 0 else float("inf")
        notes = ""
 
        if freq > nyquist * ALIASING_THRESHOLD:
            # Signal is very close to or above Nyquist — aliasing likely.
            aliased_true = fps - freq
            notes = f"⚠ aliasing? true freq ≈ {aliased_true:.1f} Hz"
 
        print(f"{rank:<6} {freq:<14.2f} {mag:<14.4f} {period_ms:<14.1f} {notes}")
        results.append(
            {
                "rank": rank,
                "frequency_hz": round(freq, 4),
                "magnitude": round(float(mag), 6),
                "period_ms": round(period_ms, 3),
                "aliasing_warning": bool(notes),
            }
        )
 
    print(f"{'=' * col}\n")
 
    # ------------------------------------------------------------------ #
    # Phase 6 – wavelet analysis                                           #
    #                                                                      #
    # The FFT assumes the signal is stationary (same frequencies the whole #
    # time). The CWT makes no such assumption — it shows how frequency     #
    # content changes over time, which lets us distinguish:                #
    #   • A real persistent vibration  → strong, stable horizontal band   #
    #   • A quantisation/flow artifact → intermittent, patchy blobs        #
    #   • Two real sources             → two distinct horizontal bands     #
    # ------------------------------------------------------------------ #
    print("Computing continuous wavelet transform (CWT) …")
 
    # Frequency axis we want to resolve — 1 Hz steps up to Nyquist.
    cwt_freqs = np.linspace(1.0, nyquist, int(nyquist))
 
    # PyWavelets uses scales; convert frequencies → scales for cmor wavelet.
    # cmor1.5-1.0 = complex Morlet with bandwidth=1.5, centre freq=1.0 Hz.
    import pywt
    wavelet = "cmor1.5-1.0"
    centre_freq = pywt.central_frequency(wavelet)
    scales = centre_freq * fps / cwt_freqs  # scale = f_c * fs / f_target
 
    # CWT on the detrended, normalised (but un-windowed) signal so we don't
    # taper the edges and lose amplitude information at the start/end.
    coeffs, _ = pywt.cwt(sig, scales, wavelet, sampling_period=1.0 / fps)
    power = np.abs(coeffs) ** 2  # shape: (n_freqs, n_samples)
 
    # Time axis for the scalogram.
    t_axis = np.arange(len(sig)) / fps
 
    # ── Scalogram plot ──────────────────────────────────────────────────
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
 
    fig, axes = plt.subplots(
        3, 1,
        figsize=(14, 12),
        gridspec_kw={"height_ratios": [1, 2, 1]},
    )
    fig.suptitle("Vibration analysis — wavelet & FFT comparison", fontsize=13)
 
    # Panel 1: raw signal
    axes[0].plot(t_axis, sig, lw=0.5, color="steelblue")
    axes[0].set_ylabel("Displacement\n(normalised)")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_title("Detrended displacement signal")
    axes[0].set_xlim(t_axis[0], t_axis[-1])
 
    # Panel 2: scalogram (power in dB, log-normalised)
    power_db = 10 * np.log10(power + 1e-12)
    vmin, vmax = np.percentile(power_db, [5, 99])
    im = axes[1].imshow(
        power_db,
        aspect="auto",
        origin="lower",
        extent=[t_axis[0], t_axis[-1], cwt_freqs[0], cwt_freqs[-1]],
        cmap="inferno",
        vmin=vmin,
        vmax=vmax,
    )
    plt.colorbar(im, ax=axes[1], label="Power (dB)")
    axes[1].set_ylabel("Frequency (Hz)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title(
        "Scalogram — persistent horizontal bands = real vibrations; "
        "patchy blobs = artifacts"
    )
    # Mark the FFT-detected peaks on the scalogram
    for freq in top_freqs:
        axes[1].axhline(freq, color="cyan", lw=0.8, ls="--", alpha=0.7)
 
    # Panel 3: time-averaged wavelet spectrum vs FFT — side-by-side comparison
    mean_power = np.mean(power, axis=1)
    mean_power /= np.max(mean_power) if np.max(mean_power) > 0 else 1.0
 
    axes[2].plot(cwt_freqs, mean_power, color="darkorange", lw=1.5, label="Wavelet (time-avg)")
    axes[2].plot(freqs_pos, spectrum_pos, color="steelblue", lw=1.0, alpha=0.7, label="FFT")
    axes[2].set_xlabel("Frequency (Hz)")
    axes[2].set_ylabel("Normalised magnitude")
    axes[2].set_title("Time-averaged wavelet power vs FFT spectrum")
    axes[2].legend()
    axes[2].set_xlim(0, nyquist)
 
    plt.tight_layout()
    diag_path = video_path.with_suffix(".wavelet.png")
    plt.savefig(diag_path, dpi=150)
    plt.close()
    print(f"Wavelet scalogram saved to: {diag_path}")
 
    # ── Wavelet-derived frequency table ─────────────────────────────────
    # Find peaks in the time-averaged wavelet power spectrum.
    from scipy.signal import find_peaks
 
    peaks_idx, props = find_peaks(
        mean_power,
        height=0.1,          # ignore anything below 10% of max
        distance=int(len(cwt_freqs) * 0.03),  # peaks must be >3% of freq range apart
        prominence=0.05,
    )
 
    col = 72
    print(f"\n{'=' * col}")
    print(f"  WAVELET-DERIVED FREQUENCIES  (Nyquist = {nyquist:.1f} Hz)")
    print(f"{'=' * col}")
    print(f"{'Rank':<6} {'Freq (Hz)':<14} {'Power':<14} {'Period (ms)':<14} Notes")
    print(f"{'-' * col}")
 
    # Sort peaks by power descending.
    sorted_peaks = sorted(peaks_idx, key=lambda i: mean_power[i], reverse=True)
    for rank, idx in enumerate(sorted_peaks[:top_n], 1):
        freq = cwt_freqs[idx]
        pwr = mean_power[idx]
        period_ms = 1000.0 / freq
        notes = ""
        if freq > nyquist * ALIASING_THRESHOLD:
            notes = f"⚠ aliasing? true freq ≈ {fps - freq:.1f} Hz"
        print(f"{rank:<6} {freq:<14.1f} {pwr:<14.4f} {period_ms:<14.1f} {notes}")
 
    print(f"{'=' * col}\n")
 
    # ── Stationarity check ───────────────────────────────────────────────
    # For each FFT-detected peak, check whether its wavelet power is stable
    # over time or intermittent. A coefficient of variation (CV) > 1.0 is a
    # strong sign of a non-stationary (possibly artifactual) source.
    print("Stationarity check (CV > 1.0 suggests intermittent / artifact):")
    print(f"  {'Freq (Hz)':<14} {'Mean power':<14} {'Std power':<14} {'CV':<10} Verdict")
    print(f"  {'-' * 60}")
    for freq in sorted(top_freqs):
        # Find nearest CWT frequency bin.
        bin_idx = np.argmin(np.abs(cwt_freqs - freq))
        time_series_power = power[bin_idx, :]
        mean_p = np.mean(time_series_power)
        std_p = np.std(time_series_power)
        cv = std_p / mean_p if mean_p > 0 else float("inf")
        verdict = "✓ stationary (real)" if cv < 1.0 else "⚠ intermittent (possible artifact)"
        print(f"  {freq:<14.1f} {mean_p:<14.4f} {std_p:<14.4f} {cv:<10.3f} {verdict}")
 
    print()
 

if __name__ == "__main__":
    APP()