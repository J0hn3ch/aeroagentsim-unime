"""
Drone XXXX - Dott. Gianluca Carbone - Thesis Master Degree Engineering and Computer Science 2025/2026

Scenario Overview: 
Two swarm of drones executes must accomplish a mission. If the swarm is not big enough to do the mission, some drones can move from a swarm to the other to help.

"""
import aeroagentsim
from aeroagentsim.core.environment import Environment
from aeroagentsim.agent import DroneAgent, DeliveryDroneAgent
from aeroagentsim.agent import TerminalAgent
from aeroagentsim.agent.delivery_station import DeliveryStation

# Components
from aeroagentsim.component import ChargingComponent, CommunicationComponent, LogisticsComponent, MoveToComponent

# Workflows
from aeroagentsim.workflow.inspection import create_inspection_workflow
from aeroagentsim.workflow.inspection import InspectionWorkflow
from aeroagentsim.workflow.logistics import LogisticsWorkflow

from aeroagentsim.core.trigger import TimeTrigger, StateTrigger

# Data Integration
from aeroagentsim.dataprovider.signal import SignalDataProvider, SignalSource
from aeroagentsim.dataprovider.signal_integration import ExternalSignalSourceIntegration

# Location, Distance and other utilities
from aeroagentsim.core.utils import Location, calculate_distance
# Conversion
# from aeroagentsim.core.utils import convert_coordinate # do nothing
from aeroagentsim.core.utils import latlon_to_local, local_to_latlon
# Time
from aeroagentsim.core.utils import utm_zone_for_lon

# Logging
from aeroagentsim.utils.logging_config import get_logger

# Performance
#from aeroagentsim.utils.profiler import SimulationProfiler
from aeroagentsim.statistics.stats_collector import StatsCollector
from aeroagentsim.statistics.stats_analyzer import StatsAnalyzer
from aeroagentsim.statistics.stats_visualizer import StatsVisualizer

# Plotting
import matplotlib
matplotlib.use('Agg') # headless – no display needed
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import gc
import logging
import math
import numpy as np
import os
import pprint
import psutil
import sys
import time
import uuid

# |----- ENVIRONMENT CONFIGURATION -----|
os.environ["AEROAGENTSIM_LOG_LEVEL"] = "ERROR"
os.environ["DEFAULT_LOG_FORMAT"] = "%(asctime)s # %(message)s"
DEFAULT_LOG_FORMAT = '%(asctime)s # %(message)s'
SIMULATION_TIME = 1000
VISUAL_INTERVAL = 15

CONTRACT_TIME     = 60           # when GCS-2 issues the cross-swarm contract
CONTRACT_DEADLINE = 700          # absolute sim-time deadline for the contract
CONTRACT_REWARD   = 100.0        # credits rewarded to the accepting drone
CONTRACT_PENALTY  = 0.0          # no penalty (simplifies balance management)

SWARM_NUMBER = 2
SWARM_SIZE = 3
OUTPUT_DIR = './simulation_stats'
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Pala Nebiolo 
# Local Coordinates : 1730000.0, 4250350.0 > latlon_to_local(38.22431587716676, 15.55826378239404, ref_lat=0.0, ref_lon=0.0)
# Latitude and Longitude : 38.22431587716676, 15.55826378239404 > local_to_latlon(1730000.0, 4250350.0, 80.0)
LOCAL_ORIGIN = (1_730_000, 4_250_350, 0)
LATLON_ORIGIN = (38.22431587716676, 15.55826378239404, 0)
ox, oy, _ = LOCAL_ORIGIN

# GCS positions (ground level)
GCS1_POS = (ox - 45,   oy - 15,   0)
GCS2_POS = (ox + 45, oy - 15,   0)
GCS1_LATLON = local_to_latlon(x=GCS1_POS[0],y=GCS1_POS[1], ref_lat=LATLON_ORIGIN[0], ref_lon=LATLON_ORIGIN[1])
GCS2_LATLON = local_to_latlon(x=GCS2_POS[0],y=GCS2_POS[1], ref_lat=LATLON_ORIGIN[0], ref_lon=LATLON_ORIGIN[1])

# |----- LOGGING CONFIGURATION -----|
logging.basicConfig(level=logging.DEBUG, format=DEFAULT_LOG_FORMAT)
mlogger = logging.getLogger(name='aeroagentsim')
mlogger.setLevel(logging.DEBUG)

# |----- PERFORMANCE CONFIGURATION -----|
# Monitor memory usage
def check_memory():
    process = psutil.Process()
    memory_mb = process.memory_info().rss / 1024 / 1024
    print(f"Memory usage: {memory_mb:.1f} MB")
    print(f"Objects in memory: {len(gc.get_objects())}")

"""
profiler = SimulationProfiler(env)
profiler.start()
"""

# |----- SYSTEM INFORMATION -----|
print("\n=====| SYSTEM INFORMATION |=====")
print(f"|- Python version: {sys.version}")
print(f"|- AeroAgentSim version: {aeroagentsim.__version__}")
print(f"|- Platform: {sys.platform}")
print(f"|- Path execution: ") #{sys.path}
print("-"*40)


# |----- SIMULATION ENVIRONMENT -----|
start_time = time.time()

# Create environment
env = Environment(visual_interval=VISUAL_INTERVAL)

# |----- STATISTICS CONFIGURATION -----|
# Set up statistics collection
stats_collector = StatsCollector(
    env,
    output_dir=OUTPUT_DIR,
    agent_collector_config={
        'listen_visual_update': True,
        'collect_interval': VISUAL_INTERVAL
    }
)

logger = get_logger(__name__)
logger.setLevel(logging.ERROR)
logger.parent.setLevel(logging.ERROR)

print("| Logger - Description:")
print(f"|- logger.format: {logger.handlers}")
print(f"|- logger.name: {logger.name}")
print(f"|- logger.level: {logger.level}")
print(f"|- logger.parent: {logger.parent}")
print(f"|- logger.root: {logger.root}")


def print_task(task, info_list=None):
    task_info = {
        'agent_id': task.agent_id, 
        'battery_level': task.battery_level, 
        'component_name': task.component_name, 
        'name': task.name, 
        'priority': task.priority.name,
        'status': task.status,
        'current_position' : task.current_position, 
        'target_position': task.target_position, 
        'target_state': task.target_state,
        'progress': task.progress, 
        'distance_traveled': task.distance_traveled, 
        'total_distance': task.total_distance
    }
    h = l = ""
    if info_list == None or info_list == 'all':
        info_list = task_info.keys()

    for e in info_list:
        if e in task_info.keys():
            #for k, v in task_info.items():
            k = e
            v = task_info[k]
            i = len(str(k)) - len(str(v))
            if i < 0:
                h = h + str(k) + ' ' * abs(i)
                l = l + str(v)
            else:
                h = h + str(k)
                l = l + str(v) + ' ' * i
            h = h + ' | '
            l = l + ' | '
    return ('| ' + h, '| ' + l)

# |----- SCENARIO CONFIGURATION -----|
print("\n=====| SCENARIO |=====")

print("\n-----|  AGENTS  |-----")
# Entities list: Base Agent, DeliveryAgent, DeliveryDroneAgent, DeliveryStation,
# - DroneAgent, InspectionStation, SensingAgent, TerminalAgent

class GCSAgent(TerminalAgent):
    """Ground Control Station – can issue contracts on behalf of its swarm."""

    def __init__(self, env, agent_name, properties=None, agent_id=None):
        super().__init__(env, agent_name, properties)
        if agent_id:
            self.id = agent_id
        self.swarm_drones: list = []          # drones this GCS controls
        self.issued_contracts: list = []

    def issue_contract_to_swarm(self, target_drones, task_info,
                                reward=100.0, penalty=0.0, deadline=float("inf"),
                                description=""):
        """
        Issue a contract addressed to a list of eligible drones.
        The contract manager will record it as PENDING; the first drone
        that calls accept_contract() will execute it.
        """
        target_ids = [d.id for d in target_drones]

        # GCS needs a non-zero balance to freeze the reward
        current = self.env.contract_manager.get_agent_balance(self.id)
        if current < reward:
            self.env.contract_manager.set_agent_balance(self.id, reward + 50)
        
        contract_id = self.env.contract_manager.create_contract(
            issuer_agent_id=self.id,
            task_info=task_info,
            reward=reward,
            penalty=penalty,
            deadline=deadline,
            appointed_agent_ids=target_ids,
            description=description,
        )

        if contract_id:
            self.issued_contracts.append(contract_id)
            print(f"[t={self.env.now:.0f}] {self.name} issued contract {contract_id} "
                  f"→ eligible drones: {[d.name for d in target_drones]}")
            contract_log.append({
                "event": "issued", "contract_id": contract_id,
                "issuer": self.name, "time": self.env.now,
                "description": description,
            })
        return contract_id

# ── A. Ground Control Stations ───────────────────────────────────────────────
# Create a ground station
agent_id = f"GCS_{uuid.uuid4().hex[:8]}"
ground_station1 = env.create_agent(
    GCSAgent,
    agent_name="Ground Control Station 1",
    agent_id = agent_id,
    properties={
        'position': GCS1_POS
    }
)

agent_id = f"GCS_{uuid.uuid4().hex[:8]}"
ground_station2 = env.create_agent(
    GCSAgent,
    agent_name="Ground Control Station 2",
    agent_id = agent_id,
    properties={
        'position': GCS2_POS
    }
)
print(f"|- [Agents] {ground_station1.name} ({ground_station1.id})  pos={GCS1_POS}")
print(f"|- [Agents] {ground_station2.name} ({ground_station2.id})  pos={GCS2_POS}")

# ── A.1 - GCS Components ───────────────────────────────────────────────

# ── B. Swarms ───────────────────────────────────────────────
print(f"|- Drones {'-'*80}")
# Create swarm of drones agents
swarm_1_size = 3
swarm_1_drones = []

for i in range(swarm_1_size):
    # Distribute drones in a grid
    x, y, _ = GCS1_POS
    x += 10 + (i % 3) * 20
    y += 10 + (i // 3) * 20
    #x = ox + 10 + (i % 3) * 40
    #y = oy + 10 + (i // 3) * 40

    # Register agent with environment
    agent_id = f"drone_{uuid.uuid4().hex[:8]}"
    drone = env.create_agent(
        DroneAgent, 
        agent_id=f"SD1_{agent_id}", 
        agent_name=f"Swarm 1/Drone {i}", 
        properties={
            'position': (x, y, 50),
            'battery_level': 100,
            'status': 'idle'
        }
    )
    #env.register_agent(drone)

    # Prefer writing logs/trajectories to run artifacts and trimming. Custom in-memory caches maintained by your own agents/components.
    drone.max_state_history = 100

    # Add components to agent
    drone.add_component(MoveToComponent(env, drone))
    drone.add_component(CommunicationComponent(env, drone))

    ground_station1.swarm_drones.append(drone)
    print(f"|- [Agents] {drone.name} (id={drone.id})  pos={(x, y, 50)}")

swarm_2_size = 3
swarm_2_drones = []

for i in range(swarm_2_size):
    # Distribute drones in a grid
    x, y, _ = GCS2_POS
    x += 10 + (i % 3) * 20
    y += 10 + (i // 3) * 20

    #x = 1730050 + (i % 3) * 40 + 40 * 4
    #y = 4250400 + (i // 3) * 40

    # Register agent with environment
    agent_id = f"drone_{uuid.uuid4().hex[:8]}"
    drone = env.create_agent(
        DroneAgent, 
        agent_id=f"SD2_{agent_id}", 
        agent_name=f"Swarm 2/Drone {i}", 
        properties={
        'position': (x, y, 50),
        'battery_level': 100,
        'status': 'idle'
        }
    )
    # env.register_agent(drone)

    # Add components to agent
    drone.add_component(MoveToComponent(env, drone))
    drone.add_component(CommunicationComponent(env, drone))

    ground_station2.swarm_drones.append(drone)
    print(f"|- [Agents] {drone.name} (id={drone.id})  pos={(x, y, 50)}")


# ── B.1 Drone Components ───────────────────────────────────────────────

# |- COMPONENTS DEFINITION
# Mobility Components: MoveToComponent
# Power Management: ChargingComponent
# Communication Components: CommunicationComponent
# Computation Components: CPUComponent, ComputationComponent
# Sensing Components: ImageSensingComponent, EMSensingComponent, ObjectSensorComponent
# Logistics Components: LogisticsComponent,

print(f"|- {drone.name} components: {drone.components}")
print(f"|- {drone.name} details: {drone.get_details()}")

# Verify component has required metrics
component = drone.get_component('MoveToComponent')
print(f"|- MoveToComponent metrics: {[metric for metric in component.current_metrics.keys()]}")

drone.add_component( ChargingComponent(env, drone) )
component = drone.get_component('ChargingComponent')
print(f"|- ChargingComponent metrics: {[metric for metric in component.current_metrics.keys()]}")

component = drone.get_component('CommunicationComponent')
print(f"|- CommunicationComponent metrics: {[metric for metric in component.current_metrics.keys()]}")

#drone.initialize_components()
print(f"|---------{'-'*80}")

"""
print("\n-----|  TASKS  |-----")

print(f"|- Task {'-'*80}")
tasks_log = []
for drone in swarm_1_drones + swarm_2_drones:
    # Execute a movement task - Tasks are managed by components. 
    task = drone.execute_task(
        component_name="MoveTo",
        task_name="Move to waypoint",
        task_class="MoveToTask",
        target_state={"position": (100, 100, 50)},
        properties={"target_position": (100, 100, 50)},
    )
    header, records = print_task(task, info_list=['agent_id','component_name','name','current_position','target_position','status'])
    tasks_log.append(records)
    
print(header)
for log in tasks_log:
    print(log)
"""

# |- WORKFLOWS DEFINITION
print("\n-----|  WORKFLOWS  |-----") # - Workflows generate tasks automatically. Use state machines to monitor agent states

#print(f"|- Workflow {'-'*80}")
workflows_log = []

# ── C.1 Workflows ───────────────────────────────────────────────
def generate_regular_hexagon(side_length=5, center=(0.0, 0.0)):
    """
    Generates (x, y) coordinates for a regular hexagon based on its center.

    Parameters:
    - side_length (float): The length of each side (equals distance from center to vertex).
    - center (tuple): The (x, y) coordinates of the hexagon's center. Default is (0.0, 0.0).

    Returns:
    - list of tuples: A list containing the (x, y) coordinates of the path.
    """
    cx, cy = center
    points = []

    # Calculate the 6 vertices (and a 7th to close the loop back to the start)
    for i in range(7):
        # 60 degrees (in radians) per vertex
        angle_rad = math.radians(60 * i)
        
        # Calculate X and Y using the center point as the origin
        x = cx + side_length * math.cos(angle_rad)
        y = cy + side_length * math.sin(angle_rad)
        
        points.append((x, y))

    return points

"""
# Define inspection points, waypoints for inspection workflow
tx = 1730000
ty = 4250400
waypoints = [
    (tx + 25, ty + 25, 100),   # Point 1
    (tx + 75, ty + 25, 100),   # Point 2
    (tx + 75, ty + 75, 100),   # Point 3
    (tx + 25, ty + 75, 100),   # Point 4
]
"""

# ── Assign own InspectionWorkflows to each swarm ─────────────────────────
print("\n|- [Setup] Creating swarm 1 inspection workflows …")

# Create inspection workflow for each drone with staggered start times
i = 1
for drone in ground_station1.swarm_drones:
    ox, oy, _ = ground_station1.get_state('position')
    waypoints = generate_regular_hexagon(
        side_length=3 + 2*i, 
        center=(ox, oy)
    )
    _ , _ , z = drone.get_state('position')
    waypoints = [pts + (z, ) for pts in waypoints]

    workflow = env.create_workflow(
        InspectionWorkflow,
        name=f"Inspection-{drone.name}",
        owner=drone,
        properties={'inspection_points': waypoints},
        start_trigger=TimeTrigger(env, trigger_time=10),  # Start at time 10
        max_starts=1
    )
    
    print(f"|- Workflow > {workflow.name}, Waypoints: {workflow.inspection_points}")
    i += 1

print("\n|- [Setup] Creating swarm 2 inspection workflows …")
# Create inspection workflow for each drone with staggered start times
i = 1
for drone in ground_station2.swarm_drones:
    
    ox, oy, _ = ground_station2.get_state('position')
    
    waypoints = generate_regular_hexagon(
        side_length=3 + 2*i, 
        center=(ox, oy)
    )

    _ , _ , z = drone.get_state('position')
    waypoints = [pts + (z, ) for pts in waypoints]
    
    workflow = env.create_workflow(
        InspectionWorkflow,
        name=f"Inspection-{drone.name}",
        owner=drone,
        properties={'inspection_points': waypoints},
        start_trigger=TimeTrigger(env, trigger_time=10),  # Start at time 10
        max_starts=1
    )

    print(f"|- Workflow > {workflow.name}, Waypoints: {workflow.inspection_points}")
    i += 1

    """
    workflow = InspectionWorkflow(
        env=env,
        name="manual_inspection",
        owner=drone,
        properties={'inspection_points': waypoints}
    )

    # Register workflow with environment (without trigger)
    env.workflow_manager.register_workflow(workflow)

    # Start workflow manually
    workflow.start()
    """


"""

# Execute a workflow
workflow = create_inspection_workflow(
    env, drone,
    [(0, 0, 100), (50, 50, 100), (100, 100, 100)],
)

# Debug workflow state
print(f"Current state: {workflow.status_machine.current_status}")
print(f"Current transition: {workflow.status_machine._get_current_transitions()}")
print(f"Valid transitions: {workflow.status_machine.state_transitions.keys()}")

# Check trigger status
for trigger in workflow.status_machine.active_triggers:
    print(f"Trigger {trigger.name}: {trigger.is_active()}")
"""
print("\n-----|  MISSION  |-----")
# Create conditional trigger: create trigger that fires when ...
#for drone in 
"""
ready_trigger = StateTrigger(
    env=env,
    agent_id=drone.id,
    state_name='battery_level',
    condition='>=',
    threshold=80,
    name="drone_ready_trigger"
)
"""

print("\n-----|  DATA INTEGRATION  |-----")
"""
# Data Provider
# Create signal provider
signal_config = {
    'propagation_model': 'free_space',
    'default_noise_floor': -100.0
}
signal_provider = SignalDataProvider(env, signal_config)

# Add signal sources
signal_source = SignalSource(
    source_id='transmitter_1',
    position=(100, 200, 50),
    center_frequency=2.4e9,  # 2.4 GHz
    transmit_power=20.0,     # 20 dBm
    bandwidth=20e6           # 20 MHz
)
signal_provider.add_signal_source(signal_source)

# Data Integration

# Create signal integration
signal_config = {
    'external_sources': [
        {
            'source_id': 'radar_1',
            'position': [1000, 2000, 100],
            'frequency': 10e9,  # 10 GHz
            'power': 30.0       # 30 dBm
        }
    ]
}
signal_integration = ExternalSignalSourceIntegration(env, signal_config)
"""

print("\n" +
    "======================\n" + 
    "| SIMULATION DETAILS |\n" +
    "======================")
existing_ids = [agent.id for agent in env.agents.values()]
print(f"| Agents: [ {existing_ids} ]")
print(f"| Provider: []")
print(f"| Workflows: [ {drone.get_active_workflows()} ]")

# Start monitoring (stats collector starts automatically)
print("\n=====| RUN THE SIMULATION |=====")
simulation_time = 1000
env.run(until=simulation_time)

# Calculate performance metrics
end_time = time.time()
real_time = end_time - start_time
print("Real Simulation Time: ", end_time)
performance_ratio = simulation_time / real_time if real_time > 0 else 0
print("Performance Ratio: ", performance_ratio)

#print_task(task, info_list='all')

print(f"Event queue size: {len(env._queue)}")
print(f"Events per second: {len(env.event_registry.events) / env.now}")
"""
print(f"Events executed: ", end='\n\t')
i = 0
for event in env.event_registry.events.keys():
    print(f"{i}. {event}", end=', ')
    i += 1
    if i % 5 == 0:
        print("", end='\n\t')
print("")


for e in env._queue:
    print(e.)
"""

# Analyze collected statistics
print("\n" +
    "======================\n" + 
    "|  REPORT STATISTICS |\n" +
    "======================")
export = stats_collector.export_data()
print(f"|- [Stats] Data exported → {export['output_dir']}")

time.sleep(5)
analyzer = StatsAnalyzer(stats_dir=export['output_dir'])

report = analyzer.generate_report()
report_filepath = export['output_dir'] + '/report_' + export['output_dir'].split('/')[-1] + '.json'
analyzer.save_report(output_file=report_filepath)
# mission completion time
# success rate
# per-agent computation time

# PLOTS
print("\n" +
    "======================\n" + 
    "|        PLOTS       |\n" +
    "======================")
# Bounding box with a margin (degrees)
"""
MARGIN = 0.003
all_lats = s1_lats + s2_lats + ex_lats + [gcs1_lat, gcs2_lat]
all_lons = s1_lons + s2_lons + ex_lons + [gcs1_lon, gcs2_lon]
BBOX = dict(
    min_lat = min(all_lats) - MARGIN,
    max_lat = max(all_lats) + MARGIN,
    min_lon = min(all_lons) - MARGIN,
    max_lon = max(all_lons) + MARGIN,
)

 
print(f"Bounding box: lat [{BBOX['min_lat']:.5f}, {BBOX['max_lat']:.5f}]  "
      f"lon [{BBOX['min_lon']:.5f}, {BBOX['max_lon']:.5f}]")
"""
visualizer = StatsVisualizer(stats_dir=export['output_dir'], report_file=report_filepath)
#visualizer.visualize_all()

# Plot styles
STYLE = {
    ground_station1.id : dict(color="#ffdd00", marker="*", markersize=100, label=ground_station1.name),
    ground_station2.id : dict(color="#00fbff", marker="*", markersize=100, label=ground_station2.name),
    'Swarm 1': { 'color': 'tab:blue' },
    'Swarm 2': { 'color': 'tab:red' }
}

gcs_s = [ground_station1, ground_station2]
swarms = [gcs.swarm_drones for gcs in gcs_s]

# ── PLOT A. Drone Initial Position ───────────────────────────────────────────
# Plot initial position of each drone
fig, ax = plt.subplots(figsize=(12, 6))

for swarm in swarms:
    position_x = []
    position_y = []
    
    for drone in swarm:
        swarm_label = drone.name.split("/")[0]
        print(f"|- Drone {drone.name} position: {drone.properties['position']}")
        position_x.append( drone.properties['position'][0] )
        position_y.append( drone.properties['position'][1] )
    # Add drones to plot
    ax.scatter(
        x=position_x, y=position_y, 
        c=STYLE[swarm_label]['color'], s=200, label=swarm_label, 
        marker='^', edgecolors='none')

# Add GCS to plot
ax.scatter(x=GCS1_POS[0], y=GCS1_POS[1], 
    c=STYLE[ground_station1.id]['color'], s=200, label=ground_station1.name, 
    marker='*', edgecolors='none'
)
ax.scatter(x=GCS2_POS[0], y=GCS2_POS[1], 
    c=STYLE[ground_station2.id]['color'], s=200, label=ground_station2.name, 
    marker='*', edgecolors='none'
)

ax.set_ylim([oy-150, oy+150])
ax.set_xlim([ox-150, ox+150])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('GCS & Drones - Local Coordinates')
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.savefig( os.path.join(OUTPUT_DIR, "A_drone_gcs_initial_position.png"), dpi=300 )
plt.close(fig)
print("  A_drone_gcs_initial_position.png ✓")

# ── PLOT B. Workflow ───────────────────────────────────────────
from matplotlib.collections import LineCollection

fig, ax = plt.subplots(figsize=(12, 6))

path_collection = []
path_latlon_collection = []
for swarm in swarms:
    for drone in swarm:
        drone_workflows = env.workflow_manager.get_agent_workflows(drone.id)

        for workflow in drone_workflows:
            path_2d = [(point[0], point[1]) for point in workflow.inspection_points]
            path_2d_latlon = [local_to_latlon(point[0], point[1])[0:2][::-1] for point in workflow.inspection_points]
            path_collection.append( path_2d )
            path_latlon_collection.append( path_2d_latlon )

        
        # Mark the drone
        #drone_position = drone.properties['position']
        drone_current_position = drone.get_current_states()['position']
        ax.plot(drone_current_position[0], drone_current_position[1], marker='^', color='black', markersize=8, label=drone.name)

colors = ["indigo", "blue", "green", "yellow", "orange", "red"]
lc = LineCollection(path_collection, linestyle='-', color=colors)
lc_latlon = LineCollection(path_latlon_collection, linestyle='-', color=colors)
ax.add_collection(lc)
ax.autoscale()

# Mark the GCSs
for gcs in gcs_s:
    gcs_position = gcs.properties['position']
    ax.plot(gcs_position[0], gcs_position[1], marker='*', color='black', markersize=8, label=gcs.name)

# Format the plot
plt.title(f"Drone Paths: Incremental Hexagon")
plt.xlabel("Local X [meters]")
plt.ylabel("Local Y [meters]")
plt.axis('equal') # Crucial: ensures the X and Y scales match so the hexagon isn't distorted
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.tight_layout()

plt.savefig( os.path.join(OUTPUT_DIR, "B_workflows.png"), dpi=300 )
plt.close(fig)
print("  B_workflow.png ✓")

# ── PLOT F. CS distance over time ───────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 6))

plt.savefig( os.path.join(OUTPUT_DIR, "F_CS_distance.png"), dpi=300 )
plt.close(fig)
print("  F_CS_distance.png ✓")

# ── PLOT G. Trajectory map (top view, X-Y) ───────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 6))

def plot_waypoints(wps, color, label, marker="D"):
    xs = [p[0]-ox for p in wps]
    ys = [p[1]-oy for p in wps]
    ax.plot(xs, ys, "--", color=color, alpha=0.4, linewidth=1)
    ax.scatter(xs, ys, color=color, marker=marker, s=60, zorder=4, label=label)

#plot_waypoints(S1_WAYPOINTS,    "#1565c0", "S1 waypoints")

plt.savefig( os.path.join(OUTPUT_DIR, "G_Trajectory_map_XY.png"), dpi=300 )
plt.close(fig)
print("  G_Trajectory_map_XY.png ✓")

# ── PLOT H. Latitude and Longitude Map - GCS and Swarm path (top view) ───────
import contextily as ctx
import geopandas as gpd
fig, ax = plt.subplots(figsize=(12, 10))

# Set axis limits to bounding box (lon/lat)
ax.set_xlim(LATLON_ORIGIN[1] - 0.001, LATLON_ORIGIN[1] + 0.001) # Longitude
ax.set_ylim(LATLON_ORIGIN[0] - 0.001, LATLON_ORIGIN[0] + 0.001) # Latitude

# Add satellite tile background
ctx.add_basemap(
    ax,
    crs="EPSG:4326", # Use crs="EPSG:4326" to work in lon/lat directly
    source=ctx.providers.Esri.WorldImagery,
    zoom="auto",
    attribution_size=6,
)

# Draw scenario elements
# Mark Ground Control Stations
for gcs in gcs_s:
    gcs_local = gcs.properties['position']
    gcs_latlon = local_to_latlon(x=gcs_local[0],y=gcs_local[1])
    style = STYLE[gcs.id]
    ax.scatter( gcs_latlon[1], gcs_latlon[0], zorder=6, 
        marker=style['marker'], s=style['markersize'],
        color=style['color'], label=style['label']
    )

# Draw Workflows Inspection Points
ax.add_collection(lc_latlon)
ax.autoscale()

# Format the plot
plt.title(f"Satellite Map")
plt.suptitle("Pala Nebiolo")
plt.xlabel("Longitude [degree]")
plt.ylabel("Latitude [degree]")
plt.axis('equal') # Crucial: ensures the X and Y scales match so the hexagon isn't distorted
plt.legend()
plt.tight_layout()

plt.savefig( os.path.join(OUTPUT_DIR, "H_Latlon_Satellite_Map_XY.png"), dpi=300, bbox_inches="tight")
plt.close(fig)
print("  H_Latlon_Satellite_Map_XY.png ✓")

# pprint.pp(report)
# Get performance report
"""
report = profiler.get_report()
print(report)
"""

"""
print(f"Events/sec: {report['events_per_second']}")
print(f"Memory usage: {report['peak_memory_mb']} MB")
"""

"""
Source:
    1. Example - Manual Workflow Startup: http://100.65.26.59:8000/examples.html#manual-workflow-startup
    2. Example - Real-time monitoring: http://100.65.26.59:8000/examples.html#real-time-monitoring

Data Integration: Weather integration, Signal integration, Traffic integration
    2. Example - Signal integration: http://100.65.26.59:8000/examples.html#signal-integration-with-automatic-management 


Best practices : 
http://100.65.26.59:8000/guides/performance_tuning.html 
"""