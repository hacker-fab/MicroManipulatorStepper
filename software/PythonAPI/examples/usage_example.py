from open_micro_stage_api import OpenMicroStageInterface
import time
import csv

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM5')
oms.read_device_state_info()



AXIS_CALIBRATION = 1
ENABLE_CALIBRATION = True
ENABLE_MOVE = False

if ENABLE_CALIBRATION:
    _, data = oms.calibrate_joint(AXIS_CALIBRATION, save_result=True)

    with open(f'output_{AXIS_CALIBRATION}.csv', 'w', newline='') as csvfile:
        # Create a CSV writer object
        writer = csv.writer(csvfile, delimiter=',')
        # Write all rows at once
        writer.writerows(data)

    import sys
    sys.exit(0)

if ENABLE_MOVE or __name__ == "__main__":
    input("press to home")
    oms.home()

    # run this once to calibrate joints
    #for i in range(3): oms.calibrate_joint(i, save_result=True)

    # home device
    input("press to 3,4,5")
    oms.set_pose(3.0, 4.0, 5.0)
    oms.wait_for_stop()

    #oms.move_to(3.1, 4.1, 5.9, f=26)
    #oms.wait_for_stop()

    input("press to 0,0,0")
    #oms.move_to(3.1, 4.1, 5.9, f=26)
    oms.set_pose(0.0, 0.0, 0.0)
    oms.wait_for_stop()

    input("press to home")
    oms.home()
    oms.wait_for_stop()

    # wait for moves to finish
    oms.read_device_state_info()
