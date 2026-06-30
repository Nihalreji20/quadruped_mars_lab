import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Aesthetic configuration
COLOR_BODY = '#313647'
COLOR_LEG = '#435663'
COLOR_PLANTED = '#A3B087'
COLOR_LIFTED = '#FFF8D4'

fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-2, 10)
ax.set_ylim(-3, 3)
ax.set_aspect('equal')
ax.set_title("Two-Phase Discontinuous Gait Simulation")
ax.grid(True, linestyle='--', alpha=0.5)

# Elements to animate
body_rect, = ax.plot([], [], 's', markersize=40, color=COLOR_BODY)
legs_lines = [ax.plot([], [], '-', lw=4, color=COLOR_LEG)[0] for _ in range(4)]
feet_markers = [ax.plot([], [], 'o', markersize=12)[0] for _ in range(4)]

# Initial positions (relative to body center)
# [Leg 1 (BL), Leg 2 (TL), Leg 3 (TR), Leg 4 (BR)]
leg_offsets = np.array([[-1, -1.5], [-1, 1.5], [1, 1.5], [1, -1.5]])

def init():
    body_rect.set_data([], [])
    for line in legs_lines: line.set_data([], [])
    for foot in feet_markers: foot.set_data([], [])
    return [body_rect] + legs_lines + feet_markers

def animate(frame):
    # Determine the state of the gait (4 distinct steps per cycle)
    cycle_step = frame % 4
    body_x = (frame // 4) * 2.0  # Body moves forward every full cycle
    
    # Base body position
    bx, by = body_x, 0
    
    # State tracking: [Leg1, Leg2, Leg3, Leg4] 
    # 0 = Planted (A3B087), 1 = Lifted/Swinging (FFF8D4)
    swing_states = [0, 0, 0, 0] 
    current_offsets = np.copy(leg_offsets)
    
    if cycle_step == 0:
        # Phase 1a: Leg 1 and 3 swing forward
        swing_states = [1, 0, 1, 0]
        current_offsets[0][0] += 1.0  # Leg 1 moves forward
        current_offsets[2][0] += 1.0  # Leg 3 moves forward
        
    elif cycle_step == 1:
        # Phase 1b: Body shifts forward (legs planted)
        bx += 1.0
        current_offsets[0][0] += 1.0
        current_offsets[2][0] += 1.0
        
    elif cycle_step == 2:
        # Phase 2a: Leg 2 and 4 swing forward
        bx += 1.0
        swing_states = [0, 1, 0, 1]
        current_offsets[0][0] += 1.0
        current_offsets[2][0] += 1.0
        current_offsets[1][0] += 1.0  # Leg 2 moves forward
        current_offsets[3][0] += 1.0  # Leg 4 moves forward
        
    elif cycle_step == 3:
        # Phase 2b: Body shifts forward again
        bx += 2.0
        current_offsets[:, 0] += 1.0
    
    # Update Body
    body_rect.set_data([bx], [by])
    
    # Update Legs and Feet
    for i in range(4):
        foot_x = bx + current_offsets[i][0]
        foot_y = by + current_offsets[i][1]
        
        legs_lines[i].set_data([bx, foot_x], [by, foot_y])
        feet_markers[i].set_data([foot_x], [foot_y])
        
        if swing_states[i] == 1:
            feet_markers[i].set_color(COLOR_LIFTED)
            feet_markers[i].set_markersize(16) # Enlarge to show lifting
        else:
            feet_markers[i].set_color(COLOR_PLANTED)
            feet_markers[i].set_markersize(12)

    return [body_rect] + legs_lines + feet_markers

ani = animation.FuncAnimation(fig, animate, init_func=init, frames=16, interval=800, blit=True)
plt.show()