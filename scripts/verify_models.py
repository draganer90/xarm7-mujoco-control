"""Cross-check Pinocchio and MuJoCo models. No claims unless this script passes."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from xarm_control.robot import load_model
from xarm_control.pin_models import PinModel

if __name__ == '__main__':
    model = load_model(ROOT)
    PinModel(ROOT, model)
