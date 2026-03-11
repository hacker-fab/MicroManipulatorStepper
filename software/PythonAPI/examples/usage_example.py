from open_micro_stage_api import OpenMicroStageInterface
import time
import csv

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM5')
oms.read_device_state_info()

_, data = oms.calibrate_joint(0, save_result=True)

with open('output_1.csv', 'w', newline='') as csvfile:
    # Create a CSV writer object
    writer = csv.writer(csvfile, delimiter=',')
    # Write all rows at once
    writer.writerows(data)


# # run this once to calibrate joints
# # for i in range(3): oms.calibrate_joint(i, save_result=True)

# input("press to home")
# oms.home()

# # run this once to calibrate joints
# #for i in range(3): oms.calibrate_joint(i, save_result=True)

# # home device
# input("press to 3,4,5")
# oms.move_to(3.0, 4.0, 5.0, f=10)
# oms.wait_for_stop()

# #oms.move_to(3.1, 4.1, 5.9, f=26)
# #oms.wait_for_stop()

# input("press to 0,0,0")
# #oms.move_to(3.1, 4.1, 5.9, f=26)
# oms.move_to(0.0, 0.0, 0.0, f=10)
# oms.wait_for_stop()

# input("press to home")
# oms.home()
# oms.wait_for_stop()

# # wait for moves to finish
# oms.read_device_state_info()
