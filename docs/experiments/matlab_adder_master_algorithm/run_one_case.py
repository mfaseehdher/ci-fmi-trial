"""Isolate one FMU case so a rejected run cannot leave an instance active."""

from __future__ import annotations

import json
import sys
from pathlib import Path


EXPERIMENT = Path(__file__).resolve().parent
REPOSITORY = EXPERIMENT.parents[2]
sys.path.insert(0, str(REPOSITORY / "src"))

from ci_fmi.master_algorithm import load_config, run_case


def main() -> int:
    try:
        run_case(load_config(sys.argv[1]))
    except Exception as error:
        print("CASE_RESULT:" + json.dumps({
            "error": type(error).__name__, "reason": str(error)
        }))
        return 1
    print("CASE_RESULT:" + json.dumps({"error": None}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
