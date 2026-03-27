from open_micro_stage_api import OpenMicroStageInterface
import time
import csv

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM5')
oms.read_device_state_info()
import os


AXIS_CALIBRATION = 2
ENABLE_CALIBRATION = False
ENABLE_MOVE = False
ENABLE_FREE_MOVE = True

if ENABLE_CALIBRATION:
    _, data = oms.calibrate_joint(AXIS_CALIBRATION, save_result=True)

    with open(f'output_{AXIS_CALIBRATION}.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerows(data)

    import sys
    sys.exit(0)

if ENABLE_MOVE:
    input("press to home or s to skip: ")
    if input().lower() != 's':
        oms.home()

    input("press to move a little bit")
    x,y,z = oms.read_current_position()
    oms.set_pose(x+0.01, y+0.01, z)
    print("Moved via software")

    input("press to 3,4,z")
    oms.set_pose(3.0, 4.0, z)
    oms.wait_for_stop()

    input("press to 0,0,0")
    oms.set_pose(0.0, 0.0, z)
    oms.wait_for_stop()

    oms.read_device_state_info()


# ---------------- NEW FREE MOVE SECTION ----------------
if ENABLE_FREE_MOVE or __name__ == "__main__":
    user_input = input("press to home or s to skip: ")
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

    oms.read_device_state_info()
