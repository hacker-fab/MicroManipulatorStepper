import csv
import math
from itertools import product

# Step sizes in mm
step_sizes_um = [1, 10, 50, 100, 250]
step_sizes_mm = [s/1000 for s in step_sizes_um]

# Feed rates to test
feed_rates = [50, 25, 10, 5, 1, 0.1]

# Generate all test points
rows = [['x', 'y', 'feed', 'delay']]  # Header

# Start with a move of 0 so we have some positional data to start with
test_points = [(0.0,0.0,10,None)]

# Track absolute position
current_x, current_y = 0.0, 0.0

test_num = 0
for feed_rate in feed_rates:
    for step_size_mm in step_sizes_mm:
        # For each step size, we'll do:
        # - X moves and back (4 points)
        # - Y moves and back (4 points)
        # - Diagonal and back (4 points)
        # - Fun draw (let's allocate ~10 points for Kirby/smiley)
        # Total: ~22 points per step size iteration
        
        # We need points_per_step_size points total, so repeat the sequence
        points_per_feed_rate = 1700 // len(feed_rates)  # ~283 points per feed rate
        points_per_step_size = points_per_feed_rate // len(step_sizes_mm)  # ~57 points per step size
        repeats = points_per_step_size // 40
        remainder = points_per_step_size % 22
        
        for repeat in range(repeats + (1 if remainder > 0 else 0)):
            # X-axis test - move step_size_mm in X direction only
            current_x += step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Move +X
            current_x += step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Back to start
            current_x -= step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Move +X again
            current_x -= step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Back to start
            
            # Y-axis test - move step_size_mm in Y direction only
            current_y += step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Move +Y
            current_y += step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Back to start
            current_y -= step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Move +Y again
            current_y -= step_size_mm
            test_points.append((current_x, current_y, feed_rate))  # Back to start
            
            # Diagonal test - move step_size_mm at 45 degrees
            diag_component = step_size_mm / math.sqrt(2)
            current_x += diag_component
            current_y += diag_component
            test_points.append((current_x, current_y, feed_rate))  # Diagonal +X+Y
            current_x += diag_component
            current_y += diag_component
            test_points.append((current_x, current_y, feed_rate))  # Back to start
            current_x -= diag_component
            current_y -= diag_component
            test_points.append((current_x, current_y, feed_rate))  # Diagonal again
            current_x -= diag_component
            current_y -= diag_component
            test_points.append((current_x, current_y, feed_rate))  # Back to start

            d = step_size_mm / math.sqrt(2)
            s = step_size_mm

            # --- LEFT EYE ---
            # up-left
            current_x -= d
            current_y += d
            test_points.append((current_x, current_y, feed_rate))

            # up-right (vertical net)
            current_x += d
            current_y += d
            test_points.append((current_x, current_y, feed_rate))

            # down-right
            current_x += d
            current_y -= d
            test_points.append((current_x, current_y, feed_rate))

            # down-left (back to center)
            current_x -= d
            current_y -= d
            test_points.append((current_x, current_y, feed_rate))


            # --- MOVE TO RIGHT EYE ---
            current_x += s
            test_points.append((current_x, current_y, feed_rate))


            # --- RIGHT EYE (same loop) ---
            current_x -= d
            current_y += d
            test_points.append((current_x, current_y, feed_rate))

            current_x += d
            current_y += d
            test_points.append((current_x, current_y, feed_rate))

            current_x += d
            current_y -= d
            test_points.append((current_x, current_y, feed_rate))

            current_x -= d
            current_y -= d
            test_points.append((current_x, current_y, feed_rate))


            # --- RETURN FROM RIGHT EYE ---
            current_x -= s
            test_points.append((current_x, current_y, feed_rate))


            # --- SMILE (symmetric arc-like V) ---
            # down-left
            current_x -= d
            current_y -= d
            test_points.append((current_x, current_y, feed_rate))

            # right
            current_x += s
            test_points.append((current_x, current_y, feed_rate))

            # right
            current_x += s
            test_points.append((current_x, current_y, feed_rate))
            
            # up-left (back to center)
            current_x -= d
            current_y += d
            test_points.append((current_x, current_y, feed_rate))
        
            # force back to 0,0
            current_x = 0.0
            current_y = 0.0
            test_points.append((current_x, current_y, feed_rate))


for point in test_points:
    rows.append([f'{point[0]:.6f}', f'{point[1]:.6f}', f'{point[2]:.1f}'])

# Part 2: 10x10 grid test (101 points including return)
grid_size = 10
grid_spacing = 0.1  # 1mm in mm units

for row_idx in range(grid_size):
    for col_idx in range(grid_size):
        x = col_idx * grid_spacing
        y = row_idx * grid_spacing
        #rows.append([f'{x:.6f}', f'{y:.6f}', '10.0', '4'])

# Final move back to start
rows.append(['0.000000', '0.000000', '10.0'])

print(f'Total rows (including header): {len(rows)}')
print(f'Total test points: {len(rows) - 1}')

# Write to CSV
with open('movement_test_no_grid.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(rows)

print('Created movement_tests.csv')