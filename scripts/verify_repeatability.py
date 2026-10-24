import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from rescue_swarm_core.mission_sim import SimulatedMissionRunner, create_scenario

scenarios = [
    "nominal",
    "failed_delivery_retry",
    "search_drone_comms_loss",
    "low_battery_allocation",
    "mission_timeout",
    "controller_exception",
    "no_survivors",
]

all_passed = True
print("=== MISSION SIMULATION REPEATABILITY AUDIT ===")
for sc in scenarios:
    cfg1 = create_scenario(sc)
    cfg1.seed = 42
    _, rep1 = SimulatedMissionRunner(cfg1).run()
    d1 = rep1.to_dict()
    # Validate JSON serialization & parsing via standard library
    json_str1 = rep1.to_json()
    parsed1 = json.loads(json_str1)
    if parsed1["scenario_name"] != d1["scenario_name"] or parsed1["status"] != d1["status"]:
        all_passed = False
        print(f"Scenario [{sc}]: JSON parsed output mismatch with report dict")

    d1.pop("wall_clock_elapsed_time_s", None)

    cfg2 = create_scenario(sc)
    cfg2.seed = 42
    _, rep2 = SimulatedMissionRunner(cfg2).run()
    d2 = rep2.to_dict()
    d2.pop("wall_clock_elapsed_time_s", None)

    identical = (d1 == d2)
    status_str = "PASS (Identical)" if identical else "FAIL (Mismatch)"
    print(f"Scenario [{sc:25s}]: {status_str} | Status: {d1['status']} | Elapsed: {d1['simulated_elapsed_time_s']}s | Incidents: {d1['incident_count']}")
    if not identical:
        all_passed = False
        print(f"  Diff: {json.dumps(d1)} vs {json.dumps(d2)}")

print(f"Overall Repeatability: {'PASSED' if all_passed else 'FAILED'}")
sys.exit(0 if all_passed else 1)
