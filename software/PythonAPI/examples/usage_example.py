from open_micro_stage_api import OpenMicroStageInterface
import time
import csv

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM9')
oms.read_device_state_info()
import os


def get_user_mode():
    """Ask user which mode to run: calibration, set move, or free move"""
    while True:
        print("\n=== Mode Selection ===")
        print("1. Calibration")
        print("2. Set Move")
        print("3. Free Move")
        print("4. Exit")
        
        choice = input("Select mode (1-4): ").strip()
        
        if choice == '1':
            return 'calibration'
        elif choice == '2':
            return 'set_move'
        elif choice == '3':
            return 'free_move'
        elif choice == '4':
            return 'exit'
        else:
            print("Invalid choice. Please select 1-4.")


def run_calibration(oms):
    """Run calibration mode"""
    try:
        axis = int(input("Enter axis to calibrate (0-2): ").strip())
        if axis not in [0, 1, 2]:
            print("Invalid axis. Must be 0, 1, or 2.")
            return
        
        _, data = oms.calibrate_joint(axis, save_result=True)
        
        with open(f'output_{axis}.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerows(data)
        
        print(f"Calibration complete. Data saved to output_{axis}.csv")
    except ValueError:
        print("Invalid input. Please enter a number.")


def run_set_move(oms):
    """Run set move mode"""
    user_input = input("Press Enter to home or 's' to skip: ")
    if user_input.lower() != 's':
        oms.home()

    input("Press Enter to move a little bit")
    x, y, z = oms.read_current_position()
    oms.set_pose(x+0.01, y+0.01, z)
    print("Moved via software")

    input("Press Enter to move to 3,4,z")
    oms.set_pose(3.0, 4.0, z)
    oms.wait_for_stop()

    input("Press Enter to move to 0,0,0")
    oms.set_pose(0.0, 0.0, z)
    oms.wait_for_stop()

    oms.read_device_state_info()


def run_free_move(oms):
    """Run free move mode"""
    user_input = input("Press Enter to home or 's' to skip: ")
    if user_input.lower() != 's':
        oms.home()

    oms.wait_for_stop()

    while True:
        try:
            x, y, z = oms.read_current_position()
            print(f"\nCurrent position -> X:{x:.4f}, Y:{y:.4f}, Z:{z:.4f}")

            user_input = input("Enter target X,Y,Z (or 'q' to quit, 'i' to info): ")

            if user_input.lower() == 'q':
                break
            if user_input.lower() == 'i':
                oms.read_device_state_info()
                continue
            
            if "+" in user_input:
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

    oms.read_device_state_info()


# Main - run once and exit
mode = get_user_mode()

if mode == 'calibration':
    run_calibration(oms)
elif mode == 'set_move':
    run_set_move(oms)
elif mode == 'free_move':
    run_free_move(oms)
