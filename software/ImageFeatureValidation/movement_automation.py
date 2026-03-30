import csv
import time
from pathlib import Path

import typer
from pypylon import pylon
from open_micro_stage_api import OpenMicroStageInterface
from PIL import Image

app = typer.Typer()


class BaslerCamera:
    """
    Basler camera interface for single image capture.
    Based on the pypylon library.
    """

    def __init__(self, camera_index=0):
        """
        Initialize the Basler camera.
        
        :param camera_index: Index of the camera to use (if multiple are connected).
        """
        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()

        if camera_index > len(devices) - 1:
            print(f"Error: Could not find Basler camera with index {camera_index}")
            print(f"Found {len(devices)} device(s)")
            self.camera = None
            return

        self.camera = pylon.InstantCamera(
            pylon.TlFactory.GetInstance().CreateDevice(devices[camera_index])
        )
        self.converter = None

    def open(self):
        """Open and configure the camera."""
        if self.camera is None:
            print("Error: Camera not initialized")
            return False

        try:
            self.camera.Open()
            print(f"Using device: {self.camera.GetDeviceInfo().GetModelName()}")

            # Configure camera settings
            self.camera.ExposureTime.Value = 8333.0
            self.camera.AcquisitionFrameRate.Value = 30.0

            # Adjust width if needed
            new_width = self.camera.Width.Value - self.camera.Width.Inc
            if new_width >= self.camera.Width.Min:
                self.camera.Width.Value = new_width

            # Start grabbing
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            # Set up image converter
            self.converter = pylon.ImageFormatConverter()
            self.converter.OutputPixelFormat = pylon.PixelType_RGB8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

            return True
        except Exception as e:
            print(f"Error opening camera: {e}")
            return False

    def close(self):
        """Close the camera."""
        if self.camera is not None and self.camera.IsOpen():
            print("Stopping camera...")
            self.camera.StopGrabbing()
            self.camera.Close()
            print("Camera closed")

    def capture_image(self):
        """
        Capture a single image from the camera.
        
        :return: Numpy array (RGB888 format) or None if capture failed.
        """
        if self.camera is None or not self.camera.IsGrabbing():
            print("Error: Camera not open or not grabbing")
            return None

        try:
            grab_result = self.camera.RetrieveResult(
                5000, pylon.TimeoutHandling_ThrowException
            )

            if grab_result.GrabSucceeded():
                image = self.converter.Convert(grab_result)
                frame = image.GetArray()
                grab_result.Release()
                return frame
            else:
                print(f"Error: {grab_result.ErrorCode} - {grab_result.ErrorDescription}")
                grab_result.Release()
                return None
        except Exception as e:
            print(f"Error capturing image: {e}")
            return None


def sanitize_filename(x, y, z):
    """
    Convert floating point coordinates to sanitized filename components.
    Replaces decimals and signs with valid characters.
    
    :param x, y, z: Floating point coordinates
    :return: Tuple of sanitized strings
    """
    def format_coord(val):
        # Replace decimal point with 'p' and minus sign with 'm'
        s = f"{val:.6f}".rstrip('0').rstrip('.')
        s = s.replace('.', 'p').replace('-', 'm')
        return s
    
    return format_coord(x), format_coord(y), format_coord(z)


def load_relative_points(csv_file):
    """
    Load relative x,y points from a CSV file.
    
    Expected CSV format (with or without header):
        x,y,feed[,delay]
        0,0,100
        1,0,100
        1,1,100,0.8
        ...
    
    The delay column is optional. If not provided, None is returned and the default delay is used.
    
    :param csv_file: Path to the CSV file
    :return: List of tuples [(x, y, feed, delay_or_none), ...]
    """
    points = []
    try:
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            # Check if first row is a header
            first_row = next(reader)
            try:
                # Try to parse as floats
                x, y, feed = map(float, first_row[:3])
                # Optional delay column
                delay = float(first_row[3]) if len(first_row) > 3 else None
                points.append((x, y, feed, delay))
            except ValueError:
                # First row is header, skip it
                pass

            # Read remaining rows
            for row in reader:
                if len(row) >= 3:
                    try:
                        x, y, feed = map(float, row[:3])
                        # Optional delay column
                        delay = float(row[3]) if len(row) > 3 else None
                        points.append((x, y, feed, delay))
                    except ValueError:
                        print(f"Warning: Skipping invalid row: {row}")
                        continue

        print(f"Loaded {len(points)} points from {csv_file}")
        return points
    except FileNotFoundError:
        print(f"Error: File not found: {csv_file}")
        return []


@app.command()
def main(
    csv_file: str,
    output: str = "captures",
    com_port: str = "COM5",
    baud_rate: int = 921600,
    camera_index: int = 0,
    delay: float = 0.5,
    screenshots: int = 1,
    show_communication: bool = False,
    show_logs: bool = False,
) -> None:
    """Move OpenMicroStageInterface stage in a pattern and capture images.
    
    Args:
        csv_file: CSV file with relative x,y coordinates, feed rate, and optional per-line delay.
                 Format: x,y,feed[,delay]
        output: Output folder for images. Defaults to "captures".
        com_port: Serial port for OpenMicroStageInterface. Defaults to "COM5".
        baud_rate: Baud rate for serial connection. Defaults to 921600.
        camera_index: Index of Basler camera to use. Defaults to 0.
        delay: Default delay in seconds between moves and captures. Defaults to 0.5.
               Can be overridden per-line via 4th CSV column.
        screenshots: Number of screenshots to take at each location. Defaults to 1.
        show_communication: Show OpenMicroStageInterface communication messages. Defaults to False.
        show_logs: Show OpenMicroStageInterface log messages. Defaults to False.
    
    The script will:
    1. Optionally home the stage
    2. Prompt for starting X,Y,Z position
    3. Move through each point keeping Z constant
    4. Capture images at each location in folder structure: idx_startx_starty_endx_endy
    5. Use per-line delay from CSV if provided, otherwise use default delay parameter
    """

    # Create output directory
    output_dir = Path(output)
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {output_dir}")

    # Load relative points
    relative_points = load_relative_points(csv_file)
    if not relative_points:
        print("Error: No points loaded from CSV file")
        return

    # Initialize OpenMicroStageInterface
    print(f"\nInitializing OpenMicroStageInterface on {com_port}...")
    oms = OpenMicroStageInterface(
        show_communication=show_communication,
        show_log_messages=show_logs
    )
    oms.connect(com_port, baud_rate)

    if oms.serial is None:
        print("Error: Failed to connect to OpenMicroStageInterface")
        return

    # Initialize camera
    print(f"\nInitializing Basler camera (index {camera_index})...")
    camera = BaslerCamera(camera_index)
    if not camera.open():
        print("Error: Failed to open camera")
        oms.disconnect()
        return

    try:
        # Prompt to home the stage
        print("\nWould you like to home the stage first?")
        home_response = input("Press 'h' to home or any other key to skip: ").strip().lower()
        if home_response == 'h':
            print("Homing stage...")
            oms.home()
            print("Stage homed successfully!")
        
        # Get or set starting position
        while True:
            try:
                x, y, z = oms.read_current_position()
                print(f"\nCurrent position -> X:{x:.4f}, Y:{y:.4f}, Z:{z:.4f}")

                user_input = input("Enter target X,Y,Z (or 's' to save starting info and continue):")

                if user_input.lower() == 's':
                    break
                x_str, y_str, z_str = user_input.split(',')
                if("+" in user_input):
                    user_input = user_input.replace("+", ",")
                x_target = float(x_str.strip())
                y_target = float(y_str.strip())
                z_target = float(z_str.strip())

                print(f"Moving to X:{x_target}, Y:{y_target}, Z:{z_target}")
                oms.set_pose(x_target, y_target, z_target)
                oms.wait_for_stop()

            except ValueError:
                print("Invalid input. Use format: X,Y,Z or X+Y+Z")
            except KeyboardInterrupt:
                print("\nExiting free move mode.")
                break
        
        # Verify position
        x_start, y_start, z_current = oms.read_current_position()
        print(f"Current position: ({x_start:.6f}, {y_start:.6f}, {z_current:.6f})")

        print(f"\nStarting movement with {len(relative_points)} points...")
        print(f"Screenshots per location: {screenshots}")
        print(f"Delay between moves: {delay}s\n")

        # Track cumulative position for folder naming (idx_startx_starty_endx_endy format)
        prev_x, prev_y = x_start, y_start
        
        # Move through each relative point and capture images
        for idx, (rel_x, rel_y, feed, line_delay) in enumerate(relative_points):
            # Use per-line delay if provided, otherwise use default
            current_delay = line_delay if line_delay is not None else delay
            
            # Calculate absolute position (XY only, keep Z constant)
            abs_x = x_start + rel_x
            abs_y = y_start + rel_y
            abs_z = z_current  # Keep Z constant

            print(f"[{idx + 1}/{len(relative_points)}] Moving to relative ({rel_x}, {rel_y}) with feed {feed}")
            print(f"  Absolute position: ({abs_x:.6f}, {abs_y:.6f}, {abs_z:.6f})")

            # Move stage
            oms.move_to(abs_x, abs_y, abs_z, feed)
            oms.wait_for_stop()

            # Read actual position
            final_x, final_y, final_z = oms.read_current_position()
            print(f"  Final position: ({final_x:.6f}, {final_y:.6f}, {final_z:.6f})")

            # Create folder with format: idx_startx_starty_endx_endy
            # Convert coordinates to filename-safe format (replace . with d)
            start_x_str = f"{prev_x:+.4f}".replace('.', 'd')
            start_y_str = f"{prev_y:+.4f}".replace('.', 'd')
            end_x_str = f"{final_x:+.4f}".replace('.', 'd')
            end_y_str = f"{final_y:+.4f}".replace('.', 'd')
            
            folder_name = f"{idx}_{start_x_str}_{start_y_str}_{end_x_str}_{end_y_str}"
            folder_path = output_dir / folder_name
            folder_path.mkdir(parents=True, exist_ok=True)
            
            # Capture multiple images distributed over the delay period
            if screenshots > 1:
                interval = current_delay / (screenshots-1)
                for shot in range(screenshots-1):
                    # Wait before capturing
                    time.sleep(interval)
                    
                    frame = camera.capture_image()
                    if frame is not None:
                        filename = f"{shot:03d}.png"
                        filepath = folder_path / filename
                        
                        img = Image.fromarray(frame, 'RGB')
                        img.save(str(filepath))
                        print(f"  Captured: {folder_name}/{filename}")
                    else:
                        print(f"  Error: Failed to capture image {shot}")
                frame = camera.capture_image()
                if frame is not None:
                    filename = f"{shot:03d}.png"
                    filepath = folder_path / filename
                        
                    img = Image.fromarray(frame, 'RGB')
                    img.save(str(filepath))
                    print(f"  Captured: {folder_name}/{filename}")
            else:
                # Single screenshot - wait for full delay then capture
                time.sleep(current_delay)
                
                frame = camera.capture_image()
                if frame is not None:
                    filename = "000.png"
                    filepath = folder_path / filename
                    
                    img = Image.fromarray(frame, 'RGB')
                    img.save(str(filepath))
                    print(f"  Captured: {folder_name}/{filename}")
                else:
                    print(f"  Error: Failed to capture image")
            
            # Update previous position for next iteration (XY only)
            prev_x, prev_y = final_x, final_y
            z_current = final_z # If it moves during other movement

        print("\nCapture complete!")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError during operation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        camera.close()
        oms.disconnect()
        print("Done")


if __name__ == '__main__':
    app()
