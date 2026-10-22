import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.control.llm_planner import LLMPlanner

planner = LLMPlanner()

# Урезанная карта — только нужное
test_map = {
    "victim_qr": {"x": -0.60, "y": 0.09, "type": "victim"},
    "start":     {"x": 0.00, "y": 0.00, "type": "start"},
    "traffic_jam": {"x": -1.60, "y": 0.80, "type": "obstacle"},
    "rubble":    {"x": -1.67, "y": 1.13, "type": "obstacle"},
}

plan = planner.get_plan(
    "Найди пострадавшего (victim_qr), считай QR-код и вернись на старт.",
    test_map
)

print(f"\n=== FINAL PLAN ===")
print(plan)
