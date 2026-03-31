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
        # FPS Control - disable and let natural speed run
        # -------------------------
        try:
            # Disable frame rate limiting
            self.camera.AcquisitionFrameRateEnable.SetValue(False)
        except:
            pass  # Feature may not be available
        
        print(f"FPS setting (before grab): {self.camera.AcquisitionFrameRate.GetValue():.1f}")

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



@APP.command()
def analyze_vibrations(
    video_path: Path,
    output_csv: Path = None,
    top_n: int = 10,
    roi_x: int = None,
    roi_y: int = None,
    roi_width: int = None,
    roi_height: int = None,
):
    """
    Analyze a video to detect vibration frequencies.
    
    Uses optical flow analysis to track motion in the video and extract
    the dominant vibration frequencies using FFT analysis.
    
    Args:
        video_path: Path to the video file to analyze
        output_csv: Optional path to save results as CSV
        top_n: Number of top frequencies to report (default: 10)
        roi_x: X coordinate of region of interest (optional)
        roi_y: Y coordinate of region of interest (optional)
        roi_width: Width of region of interest (optional)
        roi_height: Height of region of interest (optional)
    """
    
    video_path = Path(video_path)
    if not video_path.exists():
        print(f"Error: Video file not found: {video_path}")
        return
    
    print(f"Opening video: {video_path}")
    cap = cv2.VideoCapture(str(video_path))
    
    if not cap.isOpened():
        print("Error: Could not open video file")
        return
    
    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"Video properties:")
    print(f"  Duration: {frame_count / fps:.2f}s")
    print(f"  Frame rate: {fps:.1f} FPS")
    print(f"  Resolution: {width}x{height}")
    print(f"  Total frames: {frame_count}")
    
    # Validate and set ROI
    if roi_x is not None and roi_y is not None and roi_width is not None and roi_height is not None:
        roi = (roi_x, roi_y, roi_width, roi_height)
        print(f"Using ROI: x={roi_x}, y={roi_y}, w={roi_width}, h={roi_height}")
    else:
        roi = None
        print("Using full frame for analysis")
    
    print("\nAnalyzing motion in video...")
    
    # Read first frame
    ret, prev_frame = cap.read()
    if not ret:
        print("Error: Could not read first frame")
        return
    
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    if roi:
        prev_gray = prev_gray[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
    
    # Motion magnitude time series
    motion_magnitudes = []
    frame_times = []
    
    frame_idx = 1
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if roi:
                gray = gray[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
            
            # Calculate optical flow
            flow = cv2.calcOpticalFlowFarneback(
                prev_gray, gray,
                pyr_scale=0.5,
                levels=3,
                winsize=15,
                iterations=3,
                n8=True,
                poly_n=5,
                poly_sigma=1.2,
                flags=0
            )
            
            # Calculate magnitude of flow vectors
            mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            magnitude = np.mean(mag)
            
            motion_magnitudes.append(magnitude)
            frame_times.append(frame_idx / fps)
            
            prev_gray = gray
            frame_idx += 1
            
            if frame_idx % max(1, frame_count // 20) == 0:
                progress = (frame_idx / frame_count) * 100
                print(f"  Processed {frame_idx}/{frame_count} frames ({progress:.1f}%)", end='\r')
    
    except Exception as e:
        print(f"Error during analysis: {e}")
        return
    
    finally:
        cap.release()
    
    print(f"\n\nPerformed optical flow analysis on {len(motion_magnitudes)} frames")
    print(f"Motion range: {np.min(motion_magnitudes):.4f} to {np.max(motion_magnitudes):.4f}")
    
    # Normalize motion signal
    motion_array = np.array(motion_magnitudes)
    if np.std(motion_array) > 0:
        motion_normalized = (motion_array - np.mean(motion_array)) / np.std(motion_array)
    else:
        motion_normalized = motion_array
    
    # Apply Hann window to reduce spectral leakage
    window = signal.hann(len(motion_normalized))
    motion_windowed = motion_normalized * window
    
    # Compute FFT
    print("Computing FFT...")
    fft_values = scipy_fft.fft(motion_windowed)
    frequencies = scipy_fft.fftfreq(len(motion_windowed), 1/fps)
    
    # Get positive frequencies only
    positive_freq_idx = frequencies > 0
    frequencies_positive = frequencies[positive_freq_idx]
    magnitude_spectrum = np.abs(fft_values[positive_freq_idx])
    
    # Normalize magnitude spectrum
    magnitude_spectrum = magnitude_spectrum / np.max(magnitude_spectrum) if np.max(magnitude_spectrum) > 0 else magnitude_spectrum
    
    # Find top frequencies
    top_indices = np.argsort(magnitude_spectrum)[-top_n:][::-1]
    top_frequencies = frequencies_positive[top_indices]
    top_magnitudes = magnitude_spectrum[top_indices]
    
    print(f"\n{'='*70}")
    print(f"TOP {top_n} VIBRATION FREQUENCIES")
    print(f"{'='*70}")
    print(f"{'Rank':<6} {'Frequency (Hz)':<20} {'Magnitude (normalized)':<25} {'Period (ms)':<15}")
    print(f"{'-'*70}")
    
    results = []
    for rank, (freq, mag) in enumerate(zip(top_frequencies, top_magnitudes), 1):
        period_ms = (1/freq * 1000) if freq > 0 else float('inf')
        print(f"{rank:<6} {freq:<20.2f} {mag:<25.4f} {period_ms:<15.2f}")
        results.append({
            'rank': rank,
            'frequency_hz': freq,
            'magnitude': mag,
            'period_ms': period_ms,
        })
    
    print(f"{'='*70}\n")
    
    # Save results if requested
    if output_csv:
        output_csv = Path(output_csv)
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            import csv
            with open(output_csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['rank', 'frequency_hz', 'magnitude', 'period_ms'])
                writer.writeheader()
                writer.writerows(results)
            print(f"Results saved to: {output_csv}")
        except Exception as e:
            print(f"Error saving results: {e}")


if __name__ == "__main__":
    APP()
