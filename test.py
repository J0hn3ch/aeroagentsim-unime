from aeroagentsim.core.environment import Environment
from aeroagentsim.agent import DroneAgent
from aeroagentsim.agent import TerminalAgent
from aeroagentsim.component import ChargingComponent, CommunicationComponent, MoveToComponent
from aeroagentsim.workflow.inspection import InspectionWorkflow
from aeroagentsim.core.trigger import TimeTrigger

import math

LOCAL_ORIGIN = (1730000, 4250350, 0)
LATLON_ORIGIN = (38.22431587716676, 15.55826378239404, 0)
ox, oy, _ = LOCAL_ORIGIN

# GCS positions (ground level)
GCS1_POS = (ox + 0,   oy + 0,   0)
GCS2_POS = (ox + 100, oy + 0,   0)

env = Environment(visual_interval=15)

class GCSAgent(TerminalAgent):
    """Ground Control Station – can issue contracts on behalf of its swarm."""

    def __init__(self, env, agent_name, properties=None, agent_id=None):
        super().__init__(env, agent_name, properties)
        if agent_id:
            self.id = agent_id
        self.swarm_drones: list = []          # drones this GCS controls
        self.issued_contracts: list = []

ground_station1 = env.create_agent(
    GCSAgent,
    agent_name="Ground Control Station 1",
    properties={
        'position': GCS1_POS
    }
)

x, y, _ = GCS1_POS
x += 10 
y += 10

drone = env.create_agent(
        DroneAgent,  
        agent_name=f"Swarm 1/Drone 1", 
        properties={
            'position': (x, y, 50),
            'battery_level': 100,
            'status': 'idle'
        }
    )

drone.add_component(MoveToComponent(env, drone))
drone.add_component(CommunicationComponent(env, drone))

def generate_regular_hexagon(side_length=5, local_origin=(0.0, 0.0)):
    """
    Generates local (x, y) coordinates for an incremental hexagon trajectory.

    Parameters:
    - side_length: 
    - local_origin: 

    Returns:
    - list of tuples: A list containing the (x, y) coordinates of the path.
    """
    # Initialize the starting position at the local origin. Default (0, 0)
    x, y = local_origin
    points = [(x, y)]

    # Initial heading (0 radians means pointing directly right along the X-axis)
    current_angle_rad = 0.0

    # The exterior angle of a regular hexagon is 60 degrees
    turn_angle_rad = math.radians(60)

    # Draw the 6 sides of the hexagon for this lap
    for side in range(6):
        # Calculate the coordinates of the next vertex
        x += side_length * math.cos(current_angle_rad)
        y += side_length * math.sin(current_angle_rad)
        
        points.append((x, y))
        
        # Turn 60 degrees counter-clockwise for the next segment
        current_angle_rad += turn_angle_rad
    
    return points

ox, oy, _ = ground_station1.get_state('position')


waypoints = generate_regular_hexagon(
    side_length=3 + 2, 
    local_origin=(ox - 1, oy - 1)
)
"""

tx = 1730000
ty = 4250400

waypoints = [
    (tx + 25, ty + 25, 100),   # Point 1
    (tx + 75, ty + 25, 100),   # Point 2
    (tx + 75, ty + 75, 100),   # Point 3
    (tx + 25, ty + 75, 100),   # Point 4
]
"""
_ , _ , z = drone.get_state('position')
waypoints = [pts + (z, ) for pts in waypoints]
for p in waypoints:
    print(p)
    
workflow = env.create_workflow(
    InspectionWorkflow,
    name=f"Inspection-{drone.name}",
    owner=drone,
    properties={'inspection_points': waypoints},
    start_trigger=TimeTrigger(env, trigger_time=10),  # Start at time 10
    max_starts=1
)

drone_workflow_list = env.workflow_manager.get_agent_workflows(drone.id)
mermaid = []
for w in drone_workflow_list:
    mermaid.append(w.to_mermaid_diagram())
#print( dir(drone_workflow[0]) )
print(mermaid[0])
exit()

print("\n=====| RUN THE SIMULATION |=====")
simulation_time = 1000
env.run(until=simulation_time)