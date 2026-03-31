import csv
import time
from pathlib import Path
import threading

import typer
from pypylon import pylon
from open_micro_stage_api import OpenMicroStageInterface
from PIL import Image
import cv2
import numpy as np
import matplotlib.pyplot as plt

app = typer.Typer()


class CameraFeedDisplay:
    """Display live camera feed in a window."""
    
    def __init__(self, camera, window_name="Camera Feed"):
        self.camera = camera
        self.window_name = window_name
        self.running = False
        self.thread = None
    
    def start(self):
        """Start the camera feed display thread."""
        self.running = True
        self.thread = threading.Thread(target=self._display_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop the camera feed display."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        cv2.destroyAllWindows()
    
    def _display_loop(self):
        """Continuously display camera frames."""
        while self.running:
            frame = self.camera.capture_image()
            if frame is not None:
                # Convert BGR to RGB for OpenCV display
                cv2.imshow(self.window_name, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                
                # Allow window to be closed
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.running = False
            else:
                time.sleep(0.01)


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
        self.last_image = None
        self.last_image_idx = -1

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

    def get_next_image(self, image_idx):
        """Get the next captured image by index."""
        while self.last_image_idx == image_idx:
            time.sleep(0.0001)    
        return self.last_image, self.last_image_idx

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
                self.last_image = frame
                self.last_image_idx+=1
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
    Load absolute x,y points from a CSV file and convert to relative movements.
    
    Expected CSV format (with or without header):
        x,y,feed[,delay]
        0,0,100
        1,0,100
        1,1,100,0.8
        ...
    
    The delay column is optional. If not provided, None is returned and the default delay is used.
    
    :param csv_file: Path to the CSV file
    :return: List of tuples [(rel_x, rel_y, feed, delay_or_none), ...]
             where rel_x and rel_y are relative movements from previous position
    """
    points = []
    absolute_points = []
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
                absolute_points.append((x, y, feed, delay))
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
                        absolute_points.append((x, y, feed, delay))
                    except ValueError:
                        print(f"Warning: Skipping invalid row: {row}")
                        continue

        # Convert absolute points to relative movements
        prev_x, prev_y = 0.0, 0.0
        for abs_x, abs_y, feed, delay in absolute_points:
            rel_x = abs_x - prev_x
            rel_y = abs_y - prev_y
            points.append((rel_x, rel_y, abs_x, abs_y, feed, delay))
            prev_x, prev_y = abs_x, abs_y

        print(f"Loaded {len(points)} points from {csv_file}")
        return points
    except FileNotFoundError:
        print(f"Error: File not found: {csv_file}")
        return []


LAST_SCREENSHOT_IMAGE=-1

def take_screenshots(camera: BaslerCamera, delay: float, screenshots: int, folder_path: Path):
    global LAST_SCREENSHOT_IMAGE
    current_delay = delay
    if screenshots > 1:
        interval = current_delay / (screenshots-1)
        for shot in range(screenshots-1):
            # Wait before capturing
            time.sleep(interval)
            
            frame, idx = camera.get_next_image(LAST_SCREENSHOT_IMAGE)
            LAST_SCREENSHOT_IMAGE=idx
            if frame is not None:
                filename = f"{shot:03d}.png"
                filepath = folder_path / filename
                
                img = Image.fromarray(frame, 'RGB')
                img.save(str(filepath))
                print(f"  Captured: {folder_path.name}/{filename}")
            else:
                print(f"  Error: Failed to capture image {shot}")
        shot+=1
        frame, idx = camera.get_next_image(LAST_SCREENSHOT_IMAGE)
        LAST_SCREENSHOT_IMAGE=idx
        if frame is not None:
            filename = f"{shot:03d}.png"
            filepath = folder_path / filename
                
            img = Image.fromarray(frame, 'RGB')
            img.save(str(filepath))
            print(f"  Captured: {folder_path.name}/{filename}")
    else:
        # Single screenshot - wait for full delay then capture
        time.sleep(current_delay)
        
        frame, idx = camera.get_next_image(LAST_SCREENSHOT_IMAGE)
        LAST_SCREENSHOT_IMAGE=idx
        if frame is not None:
            filename = "000.png"
            filepath = folder_path / filename
            
            img = Image.fromarray(frame, 'RGB')
            img.save(str(filepath))
            print(f"  Captured: {folder_path.name}/{filename}")
        else:
            print(f"  Error: Failed to capture image")

@app.command()
def visualize_path(csv_file: str, output_file: Path = Path("movement_path_visualization.png")) -> None:
    """Visualize the movement path from a CSV file.
    
    Args:
        csv_file: CSV file with absolute x,y coordinates.
                 Format: x,y,feed[,delay]
    """
    points = load_relative_points(csv_file)
    if not points:
        print("Error: No points loaded from CSV file")
        return
    
    # Convert relative movements back to absolute coordinates for visualization
    absolute_x = [pos[2] for pos in points]
    absolute_y = [pos[3] for pos in points]
    current_x, current_y = 0.0, 0.0
    
    for idx, (rel_x, rel_y, abs_x, abs_y, feed, delay) in enumerate(points):
        current_x += rel_x
        current_y += rel_y
        assert abs(absolute_x[idx] - current_x) < 1e-9, f"X coordinate mismatch at point {idx}. Expected {absolute_x[idx]}, got {current_x}"
        assert abs(absolute_y[idx] - current_y) < 1e-9, f"Y coordinate mismatch at point {idx}. Expected {absolute_y[idx]}, got {current_y}"

    # Create the plot
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot the path
    ax.plot(absolute_x, absolute_y, 'b-', linewidth=1.5, label='Movement Path', alpha=0.7)
    
    # Plot the points
    ax.scatter(absolute_x, absolute_y, c='red', s=20, zorder=5, label='Waypoints')
    
    # Highlight start and end
    ax.scatter(absolute_x[0], absolute_y[0], c='green', s=100, marker='o', 
               zorder=6, label='Start', edgecolors='darkgreen', linewidth=2)
    ax.scatter(absolute_x[-1], absolute_y[-1], c='orange', s=100, marker='s', 
               zorder=6, label='End', edgecolors='darkorange', linewidth=2)
    
    # Add point numbers
    for i, (x, y) in enumerate(zip(absolute_x, absolute_y)):
        if i % max(1, len(absolute_x) // 20) == 0:  # Show every nth point to avoid clutter
            ax.annotate(f'{i}', (x, y), fontsize=8, alpha=0.7, 
                       xytext=(3, 3), textcoords='offset points')
    
    # Labels and formatting
    ax.set_xlabel('X Position (mm)', fontsize=12)
    ax.set_ylabel('Y Position (mm)', fontsize=12)
    ax.set_title(f'Movement Path Visualization\n({len(points)} points)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    ax.set_aspect('equal', adjustable='box')
    
    # Add statistics
    x_range = max(absolute_x) - min(absolute_x)
    y_range = max(absolute_y) - min(absolute_y)
    total_distance = sum(np.sqrt(np.diff(absolute_x)**2 + np.diff(absolute_y)**2))
    
    stats_text = f'X Range: {x_range:.4f} mm\nY Range: {y_range:.4f} mm\nTotal Distance: {total_distance:.4f} mm'
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Save figure to file
    plt.savefig(str(output_file), dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\nVisualization saved to: {output_file.absolute()}")
    print(f"Total points: {len(points)}")
    print(f"X range: {min(absolute_x):.4f} to {max(absolute_x):.4f} mm")
    print(f"Y range: {min(absolute_y):.4f} to {max(absolute_y):.4f} mm")
    print(f"Total distance: {total_distance:.4f} mm")


@app.command()
def main(
    csv_file: str,
    output: str = "captures",
    com_port: str = "COM5",
    baud_rate: int = 921600,
    camera_index: int = 0,
    delay: float = 0.5,
    screenshots: int = 3,
    show_communication: bool = False,
    show_logs: bool = False,
) -> None:
    """Move OpenMicroStageInterface stage in a pattern and capture images.
    
    Args:
        csv_file: CSV file with absolute x,y coordinates, feed rate, and optional per-line delay.
                 Format: x,y,feed[,delay]
                 Coordinates are absolute. The script converts them to relative movements.
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
    6. Convert absolute coordinates to relative movements for the stage
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

    # Initialize camera feed display
    print("Starting live camera feed display...")
    feed_display = CameraFeedDisplay(camera, "Camera Feed - Press 'q' to close window")
    feed_display.start()

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
                if("+" in user_input):
                    user_input = user_input.replace("+", ",")
                x_str, y_str, z_str = user_input.split(',')
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

        # Track cumulative position and capture locations
        prev_x, prev_y = x_start, y_start
        location_idx = 0
        idx_to_coord = []  # List of (idx, x, y) tuples
        path_order = []    # List of (start_idx, end_idx) tuples
        
        # Record starting location as index 0 and take pics
        folder_name = str(location_idx)
        folder_path = output_dir / folder_name
        folder_path.mkdir(parents=True, exist_ok=True)
        
        # Capture multiple images distributed over the delay period
        take_screenshots(camera, delay, screenshots, folder_path)
        idx_to_coord.append((location_idx, x_start, y_start))
        location_idx += 1
        
        # Move through each relative point and capture images
        for point_idx, (rel_x, rel_y,  abs_x_data, abs_y_data, feed, line_delay) in enumerate(relative_points):
            # Use per-line delay if provided, otherwise use default
            current_delay = line_delay if line_delay is not None else delay
            
            # Calculate absolute position (XY only, keep Z constant). If 0,0 go back to the original
            # starting position
            if(abs_x_data, abs_y_data) == (0, 0):
                abs_x = x_start
                abs_y = y_start
            else:
                abs_x = prev_x + rel_x
                abs_y = prev_y + rel_y
            abs_z = z_current  # Keep Z constant
            expected_distance = np.sqrt((rel_x ** 2) + (rel_y ** 2))

            print(f"[{point_idx + 1}/{len(relative_points)}] Moving to relative ({rel_x}, {rel_y}) with feed {feed}")
            print(f"  Absolute position: ({abs_x:.6f}, {abs_y:.6f}, {abs_z:.6f})")

            # Move stage
            oms.move_to(abs_x, abs_y, abs_z, feed)
            oms.wait_for_stop()

            # Read actual position
            final_x, final_y, final_z = oms.read_current_position()
            print(f"  Final position: ({final_x:.6f}, {final_y:.6f}, {final_z:.6f})")

            # Record this location
            idx_to_coord.append((location_idx, final_x-x_start, final_y-y_start))
            # Record the movement from previous location to this one
            path_order.append((location_idx - 1, location_idx, expected_distance, feed))
            
            # Create folder with simple index name
            folder_name = str(location_idx)
            folder_path = output_dir / folder_name
            folder_path.mkdir(parents=True, exist_ok=True)
            
            # Capture multiple images distributed over the delay period
            take_screenshots(camera, current_delay, screenshots, folder_path)
            
            # Update previous position for next iteration (XY only and keep z incase it moved)
            prev_x, prev_y, z_current = oms.read_current_position()
            location_idx += 1

        print("\nCapture complete!")
        
        # Save idx_to_coord.csv
        idx_to_coord_path = output_dir / "idx_to_coord.csv"
        with open(idx_to_coord_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['idx', 'x', 'y'])  # Header
            for idx, x, y in idx_to_coord:
                writer.writerow([idx, x, y])
        print(f"Saved coordinate mapping to: {idx_to_coord_path}")
        
        # Save path_order.csv
        path_order_path = output_dir / "path_order.csv"
        with open(path_order_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['start_idx', 'end_idx', 'expected_distance', 'feed'])  # Header
            for start_idx, end_idx, distance, feed_val in path_order:
                writer.writerow([start_idx, end_idx, distance, feed_val])
        print(f"Saved path order to: {path_order_path}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError during operation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        feed_display.stop()
        camera.close()
        oms.disconnect()
        print("Done")


@app.command()
def control(
    com_port: str = "COM5",
    baud_rate: int = 921600,
    camera_index: int = 0,
    show_communication: bool = False,
    show_logs: bool = False,
) -> None:
    """Interactive stage control with live camera feed.
    
    Allows manual control of the stage with homing option and real-time camera view.
    No CSV file or test functions required.
    
    Args:
        com_port: Serial port for OpenMicroStageInterface. Defaults to "COM5".
        baud_rate: Baud rate for serial connection. Defaults to 921600.
        camera_index: Index of Basler camera to use. Defaults to 0.
        show_communication: Show OpenMicroStageInterface communication messages. Defaults to False.
        show_logs: Show OpenMicroStageInterface log messages. Defaults to False.
    """

    # Initialize OpenMicroStageInterface
    print(f"Initializing OpenMicroStageInterface on {com_port}...")
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

    # Initialize camera feed display
    print("Starting live camera feed display...")
    feed_display = CameraFeedDisplay(camera, "Camera Feed - Press 'q' to close window")
    feed_display.start()

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

                user_input = input("Enter target X,Y,Z (or 'q' to quit):")

                if user_input.lower() == 'q':
                    break
                if("+" in user_input):
                    user_input = user_input.replace("+", ",")
                x_str, y_str, z_str = user_input.split(',')
                x_target = float(x_str.strip())
                y_target = float(y_str.strip())
                z_target = float(z_str.strip())

                print(f"Moving to X:{x_target}, Y:{y_target}, Z:{z_target}")
                oms.set_pose(x_target, y_target, z_target)
                oms.wait_for_stop()

            except ValueError:
                print("Invalid input. Use format: X,Y,Z or X+Y+Z")
            except KeyboardInterrupt:
                print("\nExiting control mode.")
                break
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        feed_display.stop()
        camera.close()
        oms.disconnect()
        print("Done")


if __name__ == '__main__':
    app()
