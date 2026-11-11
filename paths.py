"""Project root. Defaults to this folder; set F1_ROOT to work somewhere else."""
import os

ROOT = os.environ.get("F1_ROOT", os.path.dirname(os.path.abspath(__file__)))
