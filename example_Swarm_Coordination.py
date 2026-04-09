"""
Drone XXXX - Dott. Gianluca Carbone - Thesis Master Degree Engineering and Computer Science 2025/2026

Scenario Overview: 
Two swarm of drones executes must accomplish a mission. If the swarm is not big enough to do the mission, some drones can move from a swarm to the other to help.

"""
import aeroagentsim
from aeroagentsim.core.environment import Environment
from aeroagentsim.agent import DroneAgent, DeliveryDroneAgent
from aeroagentsim.agent.delivery_station import DeliveryStation

# Components
from aeroagentsim.component import ChargingComponent, CommunicationComponent, LogisticsComponent, MoveToComponent

# Workflows
from aeroagentsim.workflow.inspection import create_inspection_workflow
from aeroagentsim.workflow.inspection import InspectionWorkflow
from aeroagentsim.workflow.logistics import LogisticsWorkflow

from aeroagentsim.core.trigger import TimeTrigger, StateTrigger

from aeroagentsim.dataprovider.signal import SignalDataProvider, SignalSource
from aeroagentsim.dataprovider.signal_integration import ExternalSignalSourceIntegration

# Location, Distance and other utilities
from aeroagentsim.core.utils import Location, calculate_distance
# Conversion
# from aeroagentsim.core.utils import convert_coordinate # do nothing
from aeroagentsim.core.utils import latlon_to_local
from aeroagentsim.core.utils import local_to_latlon
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
import matplotlib.pyplot as plt

import gc
import logging
import numpy as np
import os
import pprint
import psutil
import sys
import time
import uuid


os.environ["AEROAGENTSIM_LOG_LEVEL"] = "ERROR"
os.environ["DEFAULT_LOG_FORMAT"] = "%(asctime)s # %(message)s"
DEFAULT_LOG_FORMAT = '%(asctime)s # %(message)s'
PLOT_OUTPUT_DIR = './simulation_stats/visualizations'

# Configure logging
logging.basicConfig(level=logging.WARNING, format=DEFAULT_LOG_FORMAT)
mlogger = get_logger(name=__name__)
mlogger.setLevel('ERROR')

# Monitor memory usage
def check_memory():
    process = psutil.Process()
    memory_mb = process.memory_info().rss / 1024 / 1024
    print(f"Memory usage: {memory_mb:.1f} MB")
    print(f"Objects in memory: {len(gc.get_objects())}")

print("\n=====| SYSTEM INFORMATION |=====")
print(f"|- Python version: {sys.version}")
print(f"|- AeroAgentSim version: {aeroagentsim.__version__}")
print(f"|- Platform: {sys.platform}")
print("-"*40)
#print(f"Path: {sys.path}")

start_time = time.time()

# Create environment
env = Environment(visual_interval=100)

# Set up statistics collection
stats_collector = StatsCollector(
    env,
    output_dir='./simulation_stats',
    agent_collector_config={
        'listen_visual_update': True,
        'collect_interval': 100
    }
)

logger = get_logger(__name__)
logger.setLevel(logging.WARNING)
logger.parent.setLevel(logging.WARNING)

print("| Logger - Description:")
print(f"|- logger.format: {logger.handlers}")
print(f"|- logger.name: {logger.name}")
print(f"|- logger.level: {logger.level}")
print(f"|- logger.parent: {logger.parent}")
print(f"|- logger.root: {logger.root}")

"""
profiler = SimulationProfiler(env)
profiler.start()
"""

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
        
    print('| ' + h)
    print('| ' + l)


print("\n=====| SCENARIO |=====")
print("\n-----|  AGENTS  |-----")
# Create swarm of drones agents
swarm_1_size = 6
swarm_1_drones = []

for i in range(swarm_1_size):
    # Distribute drones in a grid
    x = (i % 3) * 40
    y = (i // 3) * 40

    agent_id = f"drone_{uuid.uuid4().hex[:8]}"

    drone = DroneAgent(env, agent_id=f"SD1_{agent_id}", agent_name=f"S1_drone_{i}", properties={
        'position': (x, y, 100),
        'battery_level': 100
    })

    # Prefer writing logs/trajectories to run artifacts and trimming. Custom in-memory caches maintained by your own agents/components.
    drone.max_state_history = 100

    # Add components to agent
    drone.add_component(MoveToComponent(env, drone))
    drone.add_component(CommunicationComponent(env, drone))

    # Register agent with environment
    env.register_agent(drone)
    swarm_1_drones.append(drone)

swarm_2_size = 6
swarm_2_drones = []

for i in range(swarm_2_size):
    # Distribute drones in a grid
    x = (i % 3) * 40 + 40 * 3
    y = (i // 3) * 40

    agent_id = f"drone_{uuid.uuid4().hex[:8]}"

    drone = DroneAgent(env, agent_id=f"SD2_{agent_id}", agent_name=f"S2_drone_{i}", properties={
        'position': (x, y, 100),
        'battery_level': 100,
        'status': 'idle'
    })

    # Add components to agent
    drone.add_component(MoveToComponent(env, drone))
    drone.add_component(CommunicationComponent(env, drone))

    # Register agent with environment
    env.register_agent(drone)
    swarm_2_drones.append(drone)

"""
# Create agent
agent_id = f"drone_{uuid.uuid4().hex[:8]}"
drone = env.create_agent(
    DroneAgent,
    agent_id,
    properties={
        "position": [0, 0, 20],
        "battery_level": 100,
    },
) """

# Plot initial position of each drone
print(f"|- Drones {'-'*80}")
fig, ax = plt.subplots(figsize=(12, 6))

swarms = [swarm_1_drones, swarm_2_drones]
swarms.reverse()
for color in ['tab:blue', 'tab:red']:
    position_x = []
    position_y = []
    for drone in swarms.pop():
        print(f"|- Drone {drone.name} position: {drone.properties['position']}")
        position_x.append( drone.properties['position'][0] )
        position_y.append( drone.properties['position'][1] )
    ax.scatter(x=position_x, y=position_y, c=color, label="Swarm" + str(len(swarms)+1), edgecolors='none')
ax.grid(True)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Drones position')
ax.legend()
output_file = os.path.join(PLOT_OUTPUT_DIR, "drone_position_plot.png")
plt.savefig(output_file, dpi=300)
print(f"|---------{'-'*80}")

# Verify component has required metrics
component = drone.get_component('MoveToComponent')
print(f"|- MoveToComponent metrics: {[metric for metric in component.current_metrics.keys()]}")

drone.add_component( ChargingComponent(env, drone) )
# Verify component has required metrics
component = drone.get_component('ChargingComponent')
print(f"|- ChargingComponent metrics: {[metric for metric in component.current_metrics.keys()]}")

# Verify component has required metrics
component = drone.get_component('CommunicationComponent')
print(f"|- CommunicationComponent metrics: {[metric for metric in component.current_metrics.keys()]}")

#print(f"Drone details: {drone.get_details()}")
#drone.initialize_components()
print(f"|- Drone {drone.name} components: {drone.components}")

print("\n-----|  WORKFLOW  |-----")
# Execute a movement task
# - Tasks are managed by components. 
task = drone.execute_task(
    component_name="MoveTo",
    task_name="Move to waypoint",
    task_class="MoveToTask",
    target_state={"position": (100, 100, 50)},
    properties={"target_position": (100, 100, 50)},
)
print(f"|- Task {'-'*80}")
print_task(task, info_list=['agent_id','component_name','name','current_position','target_position','status'])

# Create inspection workflow for each drone with staggered start times
# Define inspection points
inspection_points = [
    (25, 25, 100),   # Point 1
    (75, 25, 100),   # Point 2
    (75, 75, 100),   # Point 3
    (25, 75, 100),   # Point 4
]


# Execute a workflow 
# - Workflows generate tasks automatically. Use state machines to monitor agent states
"""
# Define waypoints for inspection workflow
waypoints = [(50, 50, 100), (100, 0, 100), (0, 0, 100)]

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
print("| Provider: []")
print(f"| Workflows: [ {drone.get_active_workflows()} ]")

# Start monitoring (stats collector starts automatically)
print("\n=====| RUN THE SIMULATION |=====")
simulation_time = 1000
env.run(until=simulation_time)

end_time = time.time()
# Calculate performance metrics
real_time = end_time - start_time
print("Real Simulation Time: ", end_time)
performance_ratio = simulation_time / real_time if real_time > 0 else 0
print("Perfoarmance Ratio: ", performance_ratio)

print_task(task, info_list='all')

print(f"Event queue size: {len(env._queue)}")
print(f"Events per second: {len(env.event_registry.events) / env.now}")
print(f"Events executed: ", end='\n\t')
i = 0
for event in env.event_registry.events.keys():
    print(f"{i}. {event}", end=', ')
    i += 1
    if i % 5 == 0:
        print("", end='\n\t')
print("")

"""
for e in env._queue:
    print(e.)
"""


# Analyze collected statistics
print("\n" +
    "======================\n" + 
    "|  REPORT STATISTICS |\n" +
    "======================")
stats_collector.export_data()
time.sleep(5)
analyzer = StatsAnalyzer(stats_dir='./simulation_stats')
visualizer = StatsVisualizer(stats_dir='./simulation_stats')

report = analyzer.generate_report()
#visualizer.visualize_all()
pprint.pp(report)

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
"""