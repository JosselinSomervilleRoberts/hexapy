import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define the Hexapod Leg class
class HexapodLeg:
    def __init__(self, lengths):
        self.lengths = lengths  # Lengths of the leg segments [l1, l2, l3]
        
    def forward_kinematics(self, angles):
        theta1, theta2, theta3 = angles
        l1, l2, l3 = self.lengths
        
        # Rotation matrices
        R1 = np.array([
            [np.cos(theta1), 0, np.sin(theta1)],
            [0, 1, 0],
            [-np.sin(theta1), 0, np.cos(theta1)]
        ])
        
        R2 = np.array([
            [1, 0, 0],
            [0, np.cos(theta2), -np.sin(theta2)],
            [0, np.sin(theta2), np.cos(theta2)]
        ])
        
        R3 = np.array([
            [1, 0, 0],
            [0, np.cos(theta3), -np.sin(theta3)],
            [0, np.sin(theta3), np.cos(theta3)]
        ])
        
        # Position of each joint
        joint1 = R1 @ np.array([0, 0, l1])
        joint2 = joint1 + (R1 @ R2 @ np.array([0, 0, l2]))
        joint3 = joint2 + (R1 @ R2 @ R3 @ np.array([0, 0, l3]))
        
        return np.array([joint1, joint2, joint3])
    
    def inverse_kinematics(self, target):
        x, y, z = target
        l1, l2, l3 = self.lengths
        
        # Hip angle (rotation around y-axis)
        theta1 = np.arctan2(x, z)
        
        # Project target to 2D plane of knee and ankle
        r = np.hypot(x, z)
        z_prime = z - l1 * np.sin(theta1)
        r_prime = r - l1 * np.cos(theta1)
        
        # Law of cosines
        cos_theta3 = (r_prime**2 + z_prime**2 - l2**2 - l3**2) / (2 * l2 * l3)
        theta3 = np.arccos(np.clip(cos_theta3, -1.0, 1.0))
        
        # Knee angle (rotation around x-axis)
        k1 = l2 + l3 * np.cos(theta3)
        k2 = l3 * np.sin(theta3)
        theta2 = np.arctan2(z_prime, r_prime) - np.arctan2(k2, k1)
        
        return np.array([theta1, theta2, theta3])
    
    def get_joint_positions(self, angles):
        theta1, theta2, theta3 = angles
        l1, l2, l3 = self.lengths
        
        # Rotation matrices
        R1 = np.array([
            [np.cos(theta1), 0, np.sin(theta1)],
            [0, 1, 0],
            [-np.sin(theta1), 0, np.cos(theta1)]
        ])
        
        R2 = np.array([
            [1, 0, 0],
            [0, np.cos(theta2), -np.sin(theta2)],
            [0, np.sin(theta2), np.cos(theta2)]
        ])
        
        R3 = np.array([
            [1, 0, 0],
            [0, np.cos(theta3), -np.sin(theta3)],
            [0, np.sin(theta3), np.cos(theta3)]
        ])
        
        # Position of each joint
        joint1 = R1 @ np.array([0, 0, l1])
        joint2 = joint1 + (R1 @ R2 @ np.array([0, 0, l2]))
        joint3 = joint2 + (R1 @ R2 @ R3 @ np.array([0, 0, l3]))
        
        return np.array([[0, 0, 0], joint1, joint2, joint3])

# Generate circular waypoints for the end-effector
def generate_circular_waypoints(center, radius, num_points):
    angles = np.linspace(0, 2 * np.pi, num_points)
    waypoints = [[center[0] + radius * np.sin(angle), center[1] + radius * np.cos(angle), center[2]] for angle in angles]
    return waypoints

# Set up the leg lengths and circular waypoints
leg_lengths = [0.5, 0.75, 1]
center = [0,-0.5,1]
radius = 0.5
num_points = 100
waypoints = generate_circular_waypoints(center, radius, num_points)

# Create a single hexapod leg and compute IK for each waypoint
leg = HexapodLeg(leg_lengths)
ik_solutions = [leg.inverse_kinematics(target) for target in waypoints]

# Set up the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Function to update the plot
def update(num, leg, solutions, line):
    ax.cla()  # Clear the plot
    positions = leg.get_joint_positions(solutions[num])
    
    # Plot each segment of the leg
    ax.plot([0, positions[1, 0]], [0, positions[1, 1]], [0, positions[1, 2]], 'k-')
    ax.plot([positions[1, 0], positions[2, 0]], [positions[1, 1], positions[2, 1]], [positions[1, 2], positions[2, 2]], 'k-')
    ax.plot([positions[2, 0], positions[3, 0]], [positions[2, 1], positions[3, 1]], [positions[2, 2], positions[3, 2]], 'k-')
    
    # Plot the joints
    line.set_data(positions[:, 0], positions[:, 1])
    line.set_3d_properties(positions[:, 2])
    
    # Plot the base
    ax.scatter(0, 0, 0, color='red', s=100, label='Base')
    
    # Set plot limits and labels
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Hexapod Leg Kinematics')
    ax.legend()

# Initial plot setup
positions = leg.get_joint_positions(ik_solutions[0])
line, = ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'bo-', label='Leg')

# Create animation
ani = FuncAnimation(fig, update, frames=len(ik_solutions), fargs=(leg, ik_solutions, line), interval=50, repeat=True)

# Add legend
ax.legend()

plt.show()