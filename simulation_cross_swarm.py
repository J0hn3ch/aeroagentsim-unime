"""
=============================================================================
 Cross-Swarm Collaborative Inspection Simulation
=============================================================================

Scenario
--------
Two independent swarms of drones (Swarm-1, Swarm-2) are each managed by a
dedicated Ground Control Station (GCS-1, GCS-2).

  - Swarm-1 and Swarm-2 each begin their own InspectionWorkflow over a set
    of waypoints.
  - At simulation time T=CONTRACT_TIME, GCS-2 realises it needs extra help
    to complete a separate, larger inspection zone. It issues a CONTRACT
    addressed to eligible drones in Swarm-1.
  - The first idle drone of Swarm-1 that has enough battery accepts the
    contract and executes the InspectionWorkflow requested by GCS-2.
  - All drones carry a CommunicationComponent; link quality (RSSI proxy),
    distance travelled, task completion time, and workflow success/failure
    are recorded throughout.

Metrics collected
-----------------
  1. Communication link quality (signal_strength, communication_quality)
     per drone over time.
  2. Workflow success / failure rate per swarm.
  3. Task (waypoint) completion time for each drone.
  4. Distance travelled per drone.

Post-simulation plots (saved to ./sim_output/)
-----------------------------------------------
  A. Battery level over time (all drones)
  B. Communication quality over time (all drones)
  C. Distance travelled – bar chart per drone
  D. Workflow status summary – stacked bar (success/fail/pending)
  E. Task completion times – scatter per drone
  F. Signal strength over time (GCS ↔ drone links)

=============================================================================
"""

# ---------------------------------------------------------------------------
# 0.  Imports
# ---------------------------------------------------------------------------
import os, time, uuid, logging, json, math
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')           # headless – no display needed
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# AeroAgentSim core
from aeroagentsim.core.environment  import Environment
from aeroagentsim.core.trigger      import TimeTrigger, EventTrigger, StateTrigger
from aeroagentsim.core.enums        import TriggerOperator, ContractStatus

# Agents
from aeroagentsim.agent             import DroneAgent
from aeroagentsim.agent.terminal    import TerminalAgent

# Components
from aeroagentsim.component         import MoveToComponent, CommunicationComponent, ChargingComponent

# Workflows
from aeroagentsim.workflow.inspection import InspectionWorkflow

# Statistics
from aeroagentsim.statistics.stats_collector  import StatsCollector
from aeroagentsim.statistics.stats_analyzer   import StatsAnalyzer
from aeroagentsim.statistics.stats_visualizer import StatsVisualizer

from aeroagentsim.utils.logging_config import get_logger

# ---------------------------------------------------------------------------
# 1.  Configuration
# ---------------------------------------------------------------------------
os.environ["AEROAGENTSIM_LOG_LEVEL"] = "ERROR"   # keep console clean
OUTPUT_DIR        = "./sim_output"
SIMULATION_TIME   = 800          # simulated seconds
VISUAL_INTERVAL   = 50           # stats snapshot interval
CONTRACT_TIME     = 60           # when GCS-2 issues the cross-swarm contract
CONTRACT_DEADLINE = 700          # absolute sim-time deadline for the contract
CONTRACT_REWARD   = 100.0        # credits rewarded to the accepting drone
CONTRACT_PENALTY  = 0.0          # no penalty (simplifies balance management)
SWARM_SIZE        = 3            # drones per swarm

# Reference origin (local Cartesian, metres)
ORIGIN = (1_730_000, 4_250_350, 0)

os.makedirs(OUTPUT_DIR, exist_ok=True)

# Configure logging
logging.basicConfig(level=logging.ERROR, format="%(asctime)s | %(message)s")
logger = get_logger(__name__)
logger.setLevel(logging.ERROR)
logger.parent.setLevel(logging.ERROR)

# ---------------------------------------------------------------------------
# 2.  Waypoint sets
# ---------------------------------------------------------------------------
ox, oy, _ = ORIGIN

# Swarm-1 patrols a small northern zone
S1_WAYPOINTS = [
    (ox + 25,  oy + 25, 80),
    (ox + 75, oy + 25, 80),
    (ox + 75, oy + 75, 80),
    (ox + 25,  oy + 75, 80),
]

# Swarm-2 patrols a small southern zone
S2_WAYPOINTS = [
    (ox + 50, oy + 50,  80),
    (ox + 100, oy + 50,  80),
    (ox + 100, oy + 100, 80),
    (ox + 50, oy + 100, 80),
]

# Extra inspection zone that GCS-2 asks Swarm-1 help with (larger area)
EXTRA_WAYPOINTS = [
    (ox - 25,  oy - 25, 90),
    (ox - 75, oy - 25, 90),
    (ox - 75, oy - 75, 90),
    (ox - 25,  oy - 75, 90),
    (ox - 100, oy - 50, 90),
]

# GCS positions (ground level)
GCS1_POS = (ox + 0,   oy + 0,   0)
GCS2_POS = (ox + 100, oy + 0,   0)

# ---------------------------------------------------------------------------
# 3.  Metric accumulators (populated by event callbacks)
# ---------------------------------------------------------------------------
timeline: dict = defaultdict(lambda: defaultdict(list))  # [agent_id][metric] -> [(t, v)]
task_events: list  = []     # {agent_id, task_name, event, time}
workflow_results: list = []  # {workflow_id, agent_id, status, start_t, end_t}
contract_log: list = []      # one entry per contract lifecycle event

# ---------------------------------------------------------------------------
# 4.  Helper – attach a metric snapshot listener to a drone
# ---------------------------------------------------------------------------
def attach_metric_listener(env, drone: DroneAgent):
    """Subscribe to visual_update to snapshot communication metrics."""

    def on_visual_update(_event_data):
        t = env.now
        comm = drone.get_component("Communication")
        if comm and comm.current_metrics:
            for k, v in comm.current_metrics.items():
                if v is not None:
                    timeline[drone.id][k].append((t, v))
        # always record battery and distance
        batt = drone.get_state("battery_level")
        dist = drone.get_state("distance_traveled") or 0.0
        if batt is not None:
            timeline[drone.id]["battery_level"].append((t, batt))
        timeline[drone.id]["distance_traveled"].append((t, dist))

    env.event_registry.subscribe(
        env.id, "visual_update",
        f"metric_listener_{drone.id}",
        on_visual_update
    )

# ---------------------------------------------------------------------------
# 5.  Helper – attach task-event listener to a drone
# ---------------------------------------------------------------------------
def attach_task_listener(drone: DroneAgent):
    def on_task_completed(event_data):
        task_events.append({
            "agent_id":  drone.id,
            "agent_name": drone.name,
            "task_name": event_data.get("task_name", "?"),
            "status":    "completed",
            "time":      event_data.get("time", 0),
        })

    def on_task_failed(event_data):
        task_events.append({
            "agent_id":  drone.id,
            "agent_name": drone.name,
            "task_name": event_data.get("task_name", "?"),
            "status":    "failed",
            "time":      event_data.get("time", 0),
        })

    drone.env.event_registry.subscribe(drone.id, "task_completed",
        f"task_ok_{drone.id}", on_task_completed)
    drone.env.event_registry.subscribe(drone.id, "task_failed",
        f"task_fail_{drone.id}", on_task_failed)

# ---------------------------------------------------------------------------
# 6.  Helper – monitor workflow lifecycle
# ---------------------------------------------------------------------------
def attach_workflow_listener(env, workflow, swarm_label: str, drone: DroneAgent):
    wf_meta = {"workflow_id": workflow.id, "agent_id": drone.id,
                "agent_name": drone.name, "swarm": swarm_label,
                "start_t": None, "end_t": None, "status": "pending"}
    workflow_results.append(wf_meta)

    def on_status_changed(event_data):
        new_s = event_data.get("new_status", "")
        if new_s == "RUNNING" and wf_meta["start_t"] is None:
            wf_meta["start_t"] = env.now
        if new_s in ("COMPLETED", "FAILED", "CANCELED", "TIMEOUT"):
            wf_meta["end_t"]  = env.now
            wf_meta["status"] = new_s.lower()

    env.event_registry.subscribe(workflow.id, "workflow_status_changed",
        f"wf_listener_{workflow.id}", on_status_changed)

# ---------------------------------------------------------------------------
# 7.  GCS agent with contract-issuing logic
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# 8.  Build the simulation world
# ---------------------------------------------------------------------------
print("\n" + "="*60)
print("  Cross-Swarm Collaborative Inspection Simulation")
print("="*60)

env = Environment(visual_interval=VISUAL_INTERVAL)

# ── Statistics framework ──────────────────────────────────────────────────
stats_collector = StatsCollector(
    env,
    output_dir=OUTPUT_DIR,
    agent_collector_config={
        "listen_visual_update": True, 
        "collect_interval": VISUAL_INTERVAL
    },
)

# ── Ground Control Stations ───────────────────────────────────────────────
gcs1 = GCSAgent(env, "GCS-1", properties={"position": GCS1_POS},
                agent_id="gcs1")
gcs2 = GCSAgent(env, "GCS-2", properties={"position": GCS2_POS},
                agent_id="gcs2")

print(f"\n[Setup] GCS-1 id={gcs1.id}  pos={GCS1_POS}")
print(f"[Setup] GCS-2 id={gcs2.id}  pos={GCS2_POS}")

# ── Drone factory ─────────────────────────────────────────────────────────
def make_drone(env, name, position, agent_id, battery=100.0):
    drone = env.create_agent(
        DroneAgent,
        agent_name=name,
        agent_id=agent_id,
        properties={
            "position":        position,
            "battery_level":   battery,
            "status":          "idle",
            "moving_status":   "idle",
            "distance_traveled": 0.0,
        },
    )
    drone.add_component(MoveToComponent(env, drone))
    drone.add_component(CommunicationComponent(env, drone))
    drone.add_component(ChargingComponent(env, drone))
    drone.max_state_history = 200
    attach_metric_listener(env, drone)
    attach_task_listener(drone)
    return drone

# ── Swarm-1 (controlled by GCS-1) ────────────────────────────────────────
swarm1 = []
for i in range(SWARM_SIZE):
    x = ox + 10 + i * 50
    y = oy + 20
    uid = f"s1_drone{i}"
    d = make_drone(env, f"S1-Drone{i}", (x, y, 80), uid, battery=100.0)
    swarm1.append(d)
    gcs1.swarm_drones.append(d)

# ── Swarm-2 (controlled by GCS-2) ────────────────────────────────────────
swarm2 = []
for i in range(SWARM_SIZE):
    x = ox + 160 + i * 50
    y = oy + 20
    uid = f"s2_drone{i}"
    d = make_drone(env, f"S2-Drone{i}", (x, y, 80), uid, battery=100.0)
    swarm2.append(d)
    gcs2.swarm_drones.append(d)

print(f"\n[Setup] Swarm-1 drones: {[d.name for d in swarm1]}")
print(f"[Setup] Swarm-2 drones: {[d.name for d in swarm2]}")

# ── Assign own InspectionWorkflows to each swarm ─────────────────────────
print("\n[Setup] Creating own-swarm inspection workflows …")

for drone in swarm1:
    wf = env.create_workflow(
        InspectionWorkflow,
        name=f"S1-Inspection-{drone.name}",
        owner=drone,
        properties={"inspection_points": S1_WAYPOINTS},
        start_trigger=TimeTrigger(env, trigger_time=10),
        max_starts=1,
    )
    attach_workflow_listener(env, wf, "Swarm1", drone)
    print(f"  {drone.name}: S1 inspection workflow {wf.id}")

for drone in swarm2:
    wf = env.create_workflow(
        InspectionWorkflow,
        name=f"S2-Inspection-{drone.name}",
        owner=drone,
        properties={"inspection_points": S2_WAYPOINTS},
        start_trigger=TimeTrigger(env, trigger_time=10),
        max_starts=1,
    )
    attach_workflow_listener(env, wf, "Swarm2", drone)
    print(f"  {drone.name}: S2 inspection workflow {wf.id}")

# ---------------------------------------------------------------------------
# 9.  SimPy process: GCS-2 issues the cross-swarm contract at T=CONTRACT_TIME
# ---------------------------------------------------------------------------
def gcs2_cross_swarm_contract_process(env, gcs2_agent, swarm1_drones):
    """
    At CONTRACT_TIME GCS-2 issues a contract to eligible Swarm-1 drones.
    'Eligible' = battery >= 40 % and status != 'error'.
    Immediately after, the first eligible drone accepts and the contract
    workflow (InspectionWorkflow over EXTRA_WAYPOINTS) starts automatically.
    """
    yield env.timeout(CONTRACT_TIME)

    eligible = [
        d for d in swarm1_drones
        if (d.get_state("battery_level") or 0) >= 40
        and d.get_state("status") != "error"
    ]

    if not eligible:
        print(f"[t={env.now:.0f}] GCS-2: no eligible Swarm-1 drones — contract not issued.")
        return

    print(f"\n[t={env.now:.0f}] GCS-2 requesting help from Swarm-1.")
    print(f"  Eligible drones: {[d.name for d in eligible]}")

    # Build task_info expected by ContractManager
    task_info = {
        "id":        f"contract_task_{uuid.uuid4().hex[:6]}",
        "component": "MoveTo",
        "task_class":"MoveToTask",
        "task_name": "Cross-swarm extra inspection",
        "properties": {
            "inspection_points": EXTRA_WAYPOINTS,
            "target_position":   EXTRA_WAYPOINTS[0],   # first hop
        },
        "target_state": {"position": EXTRA_WAYPOINTS[0]},
    }

    contract_id = gcs2_agent.issue_contract_to_swarm(
        target_drones=eligible,
        task_info=task_info,
        reward=CONTRACT_REWARD,
        penalty=CONTRACT_PENALTY,
        deadline=CONTRACT_DEADLINE,
        description="Extra zone inspection requested by GCS-2",
    )

    if not contract_id:
        print(f"[t={env.now:.0f}] Contract creation failed (balance?).")
        return

    # ── Drone acceptance: first eligible drone accepts ────────────────────
    yield env.timeout(2)   # tiny delay simulating signalling round-trip

    accepted_drone = None
    for drone in eligible:
        # Give drone enough balance to cover potential penalty (here 0)
        env.contract_manager.set_agent_balance(drone.id, CONTRACT_PENALTY + 10)
        ok = env.contract_manager.accept_contract(contract_id, drone.id)
        if ok:
            accepted_drone = drone
            break

    if not accepted_drone:
        print(f"[t={env.now:.0f}] No drone could accept the contract.")
        return

    print(f"[t={env.now:.0f}] {accepted_drone.name} accepted contract {contract_id}.")
    contract_log.append({
        "event": "accepted", "contract_id": contract_id,
        "executor": accepted_drone.name, "time": env.now,
    })

    # ── Launch the extra InspectionWorkflow on the accepted drone ─────────
    # The ContractWorkflow (created internally by ContractManager) handles
    # the tasks declared in task_info (a single MoveToTask to the first wp).
    # For the full multi-waypoint inspection we additionally create an
    # InspectionWorkflow owned by the accepted drone, starting immediately.
    yield env.timeout(1)

    extra_wf = env.create_workflow(
        InspectionWorkflow,
        name=f"CrossSwarm-Extra-{accepted_drone.name}",
        owner=accepted_drone,
        properties={"inspection_points": EXTRA_WAYPOINTS},
        start_trigger=TimeTrigger(env, trigger_time=env.now + 1),
        max_starts=1,
    )
    attach_workflow_listener(env, extra_wf, "CrossSwarm", accepted_drone)

    print(f"[t={env.now:.0f}] Extra InspectionWorkflow {extra_wf.id} "
          f"assigned to {accepted_drone.name}.")
    contract_log.append({
        "event": "workflow_started", "contract_id": contract_id,
        "workflow_id": extra_wf.id, "drone": accepted_drone.name,
        "time": env.now,
    })

    # ── Communication monitoring: mark drone as transmitting to GCS-2 ─────
    accepted_drone.update_state("trans_target_agent_id", gcs2_agent.id)
    accepted_drone.update_state("transmitting_status", "transmitting")

    # ── Listen for extra workflow completion ──────────────────────────────
    def on_extra_wf_completed(event_data):
        new_s = event_data.get("new_status", "")
        if new_s in ("COMPLETED", "FAILED", "CANCELED"):
            contract_log.append({
                "event": f"extra_wf_{new_s.lower()}",
                "contract_id": contract_id,
                "drone": accepted_drone.name,
                "time": env.now,
            })
            print(f"\n[t={env.now:.0f}] Extra workflow {new_s} for {accepted_drone.name}.")
            # stop communication
            accepted_drone.update_state("transmitting_status", "idle")

    env.event_registry.subscribe(
        extra_wf.id, "workflow_status_changed",
        f"extra_wf_done_{extra_wf.id}", on_extra_wf_completed,
    )


env.process(gcs2_cross_swarm_contract_process(env, gcs2, swarm1))

# ---------------------------------------------------------------------------
# 10.  Communication simulation process
#      Each drone periodically updates signal_strength toward its own GCS.
# ---------------------------------------------------------------------------
def comm_simulation_process(env, drone, gcs_pos):
    """Simulate a simple free-space path-loss signal strength update."""
    while True:
        yield env.timeout(VISUAL_INTERVAL)
        pos = drone.get_state("position")
        if pos is None:
            continue
        dx = pos[0] - gcs_pos[0]
        dy = pos[1] - gcs_pos[1]
        dz = pos[2] - gcs_pos[2]
        dist_m = max(1.0, math.sqrt(dx*dx + dy*dy + dz*dz))
        # Free-space path loss (simplified, 2.4 GHz)
        freq_hz = 2.4e9
        c = 3e8
        fspl_db = 20*math.log10(dist_m) + 20*math.log10(freq_hz) - 20*math.log10(c/(4*math.pi))
        tx_power_dbm = 23.0   # 200 mW transmitter
        rssi = tx_power_dbm - fspl_db
        timeline[drone.id]["rssi_dbm"].append((env.now, rssi))
        timeline[drone.id]["gcs_distance_m"].append((env.now, dist_m))

for d in swarm1:
    env.process(comm_simulation_process(env, d, GCS1_POS))
for d in swarm2:
    env.process(comm_simulation_process(env, d, GCS2_POS))

# ---------------------------------------------------------------------------
# 11.  Run the simulation
# ---------------------------------------------------------------------------
print("\n" + "="*60)
print("  Running simulation …")
print("="*60)
wall_t0 = time.time()
env.run(until=SIMULATION_TIME)
wall_elapsed = time.time() - wall_t0

print(f"\n[Done] Simulation time = {env.now:.1f} s  |  "
      f"Wall-clock = {wall_elapsed:.2f} s  |  "
      f"Speed-up = {SIMULATION_TIME/max(wall_elapsed,0.001):.0f}×")

# ---------------------------------------------------------------------------
# 12.  Export stats
# ---------------------------------------------------------------------------
export = stats_collector.export_data()
print(f"[Stats] Data exported → {export['output_dir']}")

# ---------------------------------------------------------------------------
# 13.  Build summary structures for plotting
# ---------------------------------------------------------------------------
all_drones = swarm1 + swarm2
swarm_label = {d.id: "Swarm-1" for d in swarm1}
swarm_label.update({d.id: "Swarm-2" for d in swarm2})
drone_name  = {d.id: d.name for d in all_drones}
colors_s1   = plt.cm.Blues(np.linspace(0.4, 0.9, SWARM_SIZE))
colors_s2   = plt.cm.Reds(np.linspace(0.4, 0.9, SWARM_SIZE))
color_map   = {}
for i, d in enumerate(swarm1):  color_map[d.id] = colors_s1[i]
for i, d in enumerate(swarm2):  color_map[d.id] = colors_s2[i]

def get_series(drone_id, metric):
    pts = timeline[drone_id].get(metric, [])
    if not pts:
        return [], []
    ts, vs = zip(*pts)
    return list(ts), list(vs)

# Final distance for each drone
final_distances = {}
for d in all_drones:
    ts, vs = get_series(d.id, "distance_traveled")
    final_distances[d.id] = vs[-1] if vs else 0.0

# Workflow summary per swarm
def count_wf_statuses(swarm_label_str):
    subset = [w for w in workflow_results if w["swarm"] == swarm_label_str]
    counts = defaultdict(int)
    for w in subset:
        counts[w["status"]] += 1
    return counts

# Task completion times
completed_tasks = [e for e in task_events if e["status"] == "completed"]
failed_tasks    = [e for e in task_events if e["status"] == "failed"]

# ---------------------------------------------------------------------------
# 14.  Plotting
# ---------------------------------------------------------------------------
print("\n[Plots] Generating figures …")
os.makedirs(export['output_dir'] + '/visualizations', exist_ok=True)
FIG_W, FIG_H = 11, 4.5
PLOT_OUTPUT_DIR = export['output_dir'] + '/visualizations/'

# ── A.  Battery level over time ───────────────────────────────────────────
fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
for d in all_drones:
    ts, vs = get_series(d.id, "battery_level")
    if ts:
        ax.plot(ts, vs, color=color_map[d.id], linewidth=1.8,
                label=d.name, alpha=0.85)
ax.axvline(CONTRACT_TIME, color="black", linestyle="--", linewidth=1,
           label=f"Contract issued (t={CONTRACT_TIME})")
ax.set_xlabel("Simulation time (s)")
ax.set_ylabel("Battery level (%)")
ax.set_title("Battery Level Over Time – All Drones")
ax.set_ylim(0, 105)
ax.legend(fontsize=7, ncol=3, loc="lower left")
ax.grid(True, alpha=0.3)
s1_patch = mpatches.Patch(color=colors_s1[1], label="Swarm-1")
s2_patch = mpatches.Patch(color=colors_s2[1], label="Swarm-2")
ax.legend(handles=ax.get_lines() + [s1_patch, s2_patch],
          fontsize=7, ncol=4, loc="lower left")
plt.tight_layout()
fig.savefig(os.path.join(PLOT_OUTPUT_DIR, "A_battery_level.png"), dpi=150)
plt.close(fig)
print("  A_battery_level.png ✓")

# ── B.  Communication quality over time ───────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(FIG_W, FIG_H), sharey=False)
for ax, metric, ylabel, title in [
    (axes[0], "rssi_dbm",            "RSSI (dBm)",   "RSSI (Free-Space) Over Time"),
    (axes[1], "communication_quality","Quality (0-100)","Communication Quality Over Time"),
]:
    for d in all_drones:
        ts, vs = get_series(d.id, metric)
        if ts:
            ax.plot(ts, vs, color=color_map[d.id], linewidth=1.6,
                    label=d.name, alpha=0.85)
    ax.axvline(CONTRACT_TIME, color="black", linestyle="--", linewidth=1)
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
plt.tight_layout()
fig.savefig(os.path.join(PLOT_OUTPUT_DIR, "B_communication_quality.png"), dpi=150)
plt.close(fig)
print("  B_communication_quality.png ✓")

# ── C.  Distance travelled – bar chart ───────────────────────────────────
fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
names  = [drone_name[d.id] for d in all_drones]
dists  = [final_distances[d.id] for d in all_drones]
cols   = [color_map[d.id] for d in all_drones]
bars   = ax.bar(names, dists, color=cols, edgecolor="white", width=0.6)
for bar, val in zip(bars, dists):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
            f"{val:.1f}", ha="center", va="bottom", fontsize=8)
ax.set_xlabel("Drone")
ax.set_ylabel("Total distance (m)")
ax.set_title("Total Distance Travelled per Drone")
ax.grid(True, axis="y", alpha=0.3)
s1_patch = mpatches.Patch(color=colors_s1[1], label="Swarm-1")
s2_patch = mpatches.Patch(color=colors_s2[1], label="Swarm-2")
ax.legend(handles=[s1_patch, s2_patch])
plt.tight_layout()
fig.savefig(os.path.join(PLOT_OUTPUT_DIR, "C_distance_travelled.png"), dpi=150)
plt.close(fig)
print("  C_distance_travelled.png ✓")

# ── D.  Workflow success/failure – stacked bars per swarm ─────────────────
fig, ax = plt.subplots(figsize=(6, FIG_H))
swarm_labels_plot = ["Swarm-1", "Swarm-2", "CrossSwarm"]
status_keys = ["completed", "failed", "canceled", "pending"]
status_colors = {"completed": "#4caf50", "failed": "#f44336",
                 "canceled": "#ff9800", "pending": "#90a4ae"}
bottoms = np.zeros(len(swarm_labels_plot))
for sk in status_keys:
    heights = []
    for sl in swarm_labels_plot:
        counts = count_wf_statuses(sl)
        heights.append(counts.get(sk, 0))
    ax.bar(swarm_labels_plot, heights, bottom=bottoms,
           color=status_colors[sk], label=sk.capitalize(), edgecolor="white")
    bottoms += np.array(heights)
ax.set_ylabel("Number of workflows")
ax.set_title("Workflow Status Summary by Group")
ax.legend(fontsize=9)
ax.grid(True, axis="y", alpha=0.3)
plt.tight_layout()
fig.savefig(os.path.join(PLOT_OUTPUT_DIR, "D_workflow_summary.png"), dpi=150)
plt.close(fig)
print("  D_workflow_summary.png ✓")

# ── E.  Task completion times – scatter ───────────────────────────────────
fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
if completed_tasks:
    for e in completed_tasks:
        c = color_map.get(e["agent_id"], "gray")
        ax.scatter(e["time"], e["agent_name"], color=c, s=60, alpha=0.8, zorder=3)
if failed_tasks:
    for e in failed_tasks:
        ax.scatter(e["time"], e["agent_name"], marker="x",
                   color="red", s=80, zorder=4)
ax.axvline(CONTRACT_TIME, color="black", linestyle="--", linewidth=1,
           label=f"Contract t={CONTRACT_TIME}")
ax.set_xlabel("Simulation time (s)")
ax.set_title("Task Completion Events (● = success, ✕ = fail)")
ax.legend(fontsize=9)
ax.grid(True, axis="x", alpha=0.3)
plt.tight_layout()
fig.savefig(os.path.join(PLOT_OUTPUT_DIR, "E_task_completion_times.png"), dpi=150)
plt.close(fig)
print("  E_task_completion_times.png ✓")

# ── F.  GCS distance over time ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
for d in all_drones:
    ts, vs = get_series(d.id, "gcs_distance_m")
    if ts:
        ax.plot(ts, vs, color=color_map[d.id], linewidth=1.6,
                label=d.name, alpha=0.85)
ax.axvline(CONTRACT_TIME, color="black", linestyle="--", linewidth=1)
ax.set_xlabel("Simulation time (s)")
ax.set_ylabel("Distance to own GCS (m)")
ax.set_title("Drone–GCS Distance Over Time")
ax.legend(fontsize=7, ncol=3)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(os.path.join(PLOT_OUTPUT_DIR, "F_gcs_distance.png"), dpi=150)
plt.close(fig)
print("  F_gcs_distance.png ✓")

# ── G.  Trajectory map (top view, X-Y) ───────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 7))

def plot_waypoints(wps, color, label, marker="D"):
    xs = [p[0]-ox for p in wps]
    ys = [p[1]-oy for p in wps]
    ax.plot(xs, ys, "--", color=color, alpha=0.4, linewidth=1)
    ax.scatter(xs, ys, color=color, marker=marker, s=60, zorder=4, label=label)

plot_waypoints(S1_WAYPOINTS,    "#1565c0", "S1 waypoints")
plot_waypoints(S2_WAYPOINTS,    "#b71c1c", "S2 waypoints")
plot_waypoints(EXTRA_WAYPOINTS, "#e65100", "Extra (GCS-2 contract)", marker="*")

# GCS markers
ax.scatter([GCS1_POS[0]-ox], [GCS1_POS[1]-oy], marker="^", s=200,
           color="#1565c0", zorder=5, label="GCS-1")
ax.scatter([GCS2_POS[0]-ox], [GCS2_POS[1]-oy], marker="^", s=200,
           color="#b71c1c", zorder=5, label="GCS-2")

# Drone initial positions
for d in swarm1:
    p = d.get_state("position") or (ox, oy, 0)
    ax.scatter(p[0]-ox, p[1]-oy, color="#42a5f5", s=80, zorder=5)
for d in swarm2:
    p = d.get_state("position") or (ox, oy, 0)
    ax.scatter(p[0]-ox, p[1]-oy, color="#ef5350", s=80, zorder=5)

ax.set_xlabel("X offset (m)")
ax.set_ylabel("Y offset (m)")
ax.set_title("Scenario Map – Waypoints and Initial Positions")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(os.path.join(PLOT_OUTPUT_DIR, "G_scenario_map.png"), dpi=150)
plt.close(fig)
print("  G_scenario_map.png ✓")

# ---------------------------------------------------------------------------
# 15.  Console summary report
# ---------------------------------------------------------------------------
print("\n" + "="*60)
print("  SIMULATION SUMMARY REPORT")
print("="*60)

# Workflow results
print("\n── Workflow Results ──────────────────────────────────────")
for w in workflow_results:
    dur = ""
    if w["start_t"] is not None and w["end_t"] is not None:
        dur = f"  duration={w['end_t']-w['start_t']:.1f}s"
    print(f"  [{w['swarm']:7s}] {w['agent_name']:12s}  "
          f"status={w['status']:10s}{dur}")

# Contract log
print("\n── Contract Lifecycle ────────────────────────────────────")
for entry in contract_log:
    print(f"  [t={entry['time']:6.1f}]  {entry['event']:20s}  "
          + "  ".join(f"{k}={v}" for k, v in entry.items()
                      if k not in ("event", "time")))

# Task stats
total_ok   = len(completed_tasks)
total_fail = len(failed_tasks)
print(f"\n── Task Events ───────────────────────────────────────────")
print(f"  Completed : {total_ok}")
print(f"  Failed    : {total_fail}")
if completed_tasks:
    times = [e["time"] for e in completed_tasks]
    print(f"  Earliest completion : t={min(times):.1f}s")
    print(f"  Latest  completion  : t={max(times):.1f}s")
    print(f"  Mean completion time: t={sum(times)/len(times):.1f}s")

# Distance summary
print(f"\n── Distance Travelled ────────────────────────────────────")
for d in all_drones:
    print(f"  {d.name:14s}  {final_distances[d.id]:8.1f} m")

# Workflow success rate
all_wf = workflow_results
n_completed = sum(1 for w in all_wf if w["status"] == "completed")
n_total     = len(all_wf)
rate = 100 * n_completed / n_total if n_total else 0
print(f"\n── Workflow Success Rate ─────────────────────────────────")
print(f"  {n_completed} / {n_total} completed  ({rate:.1f} %)")

print(f"\n── Output files ──────────────────────────────────────────")
for fname in ["A_battery_level.png","B_communication_quality.png",
              "C_distance_travelled.png","D_workflow_summary.png",
              "E_task_completion_times.png","F_gcs_distance.png",
              "G_scenario_map.png"]:
    print(f"  {PLOT_OUTPUT_DIR}/{fname}")

print("\n" + "="*60)
print("  Simulation complete.")
print("="*60 + "\n")
