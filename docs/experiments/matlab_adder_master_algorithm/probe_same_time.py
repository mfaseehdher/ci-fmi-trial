"""Measure whether the MATLAB FMU updates a direct output before doStep."""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave


EXPERIMENT = Path(__file__).resolve().parent
FMUS = EXPERIMENT / "fmus"


def main() -> None:
    capabilities = {}
    for path in sorted(FMUS.glob("*.fmu")):
        description = read_model_description(path, validate=False)
        capabilities[path.name] = {
            "canGetAndSetFMUstate": bool(description.coSimulation.canGetAndSetFMUstate),
            "canBeInstantiatedOnlyOncePerProcess": bool(
                description.coSimulation.canBeInstantiatedOnlyOncePerProcess
            ),
        }

    path = FMUS / "weighted_adder.fmu"
    description = read_model_description(path, validate=False)
    references = {
        variable.name: variable.valueReference
        for variable in description.modelVariables
    }
    directory = extract(path)
    slave = FMU2Slave(
        guid=description.guid,
        unzipDirectory=directory,
        modelIdentifier=description.coSimulation.modelIdentifier,
        instanceName="same_time_probe",
    )
    instantiated = False
    try:
        slave.instantiate()
        instantiated = True
        slave.setupExperiment(startTime=0.0)
        slave.enterInitializationMode()
        slave.setReal([references["FeedbackGain"]], [0.5])
        slave.exitInitializationMode()
        slave.setReal([references["Bias"], references["Feedback"]], [1.0, 0.0])
        initial = slave.getReal([references["Out"]])[0]
        slave.setReal([references["Feedback"]], [2.0])
        before_step = slave.getReal([references["Out"]])[0]
        slave.doStep(currentCommunicationPoint=0.0, communicationStepSize=0.1)
        after_step = slave.getReal([references["Out"]])[0]
    finally:
        if instantiated:
            slave.terminate()
            slave.freeInstance()
        shutil.rmtree(directory)

    report = {
        "equation": "Out = Bias + 0.5 * Feedback",
        "before_feedback_change": initial,
        "after_set_before_doStep": before_step,
        "after_doStep": after_step,
        "supports_same_time_evaluation": not math.isclose(initial, before_step),
        "fmu_capabilities": capabilities,
    }
    results = EXPERIMENT / "results"
    results.mkdir(exist_ok=True)
    (results / "same_time_probe.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    if not (math.isclose(initial, 0.0, abs_tol=1e-9)
            and math.isclose(before_step, 0.0, abs_tol=1e-9)
            and math.isclose(after_step, 2.0, abs_tol=1e-9)):
        raise AssertionError("FMU same-time behavior changed; review Stage 1 case expectations")


if __name__ == "__main__":
    main()
