"""Check the goalkeeper rule and the printed error formula on the 18 trials."""

import csv
from pathlib import Path

from robot.parameters import (
    REGRESSION,
    RULE_ANGLE_DEG,
    RULE_DISTANCE_CM,
    RULE_SPEED_CM_S,
)

TRIALS = Path(__file__).resolve().parents[1] / "data" / "trials.csv"


def load_trials(path=TRIALS):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def predicts_stop(distance_cm, angle_deg, speed_cm_s):
    return (
        abs(angle_deg) <= RULE_ANGLE_DEG
        and speed_cm_s <= RULE_SPEED_CM_S
        and distance_cm >= RULE_DISTANCE_CM
    )


def predicted_error(distance_cm, angle_deg, speed_cm_s):
    a0, a1, a2, a3 = REGRESSION
    return a0 + a1 * distance_cm + a2 * angle_deg + a3 * speed_cm_s


def rule_scores(trials):
    correct = false_positives = false_negatives = 0
    for trial in trials:
        actual = int(trial["stopped"])
        predicted = int(predicts_stop(
            float(trial["distance_cm"]),
            float(trial["angle_deg"]),
            float(trial["speed_cm_s"]),
        ))
        correct += predicted == actual
        false_positives += predicted == 1 and actual == 0
        false_negatives += predicted == 0 and actual == 1
    return {
        "accuracy": correct / len(trials),
        "false_positives": false_positives,
        "false_negatives": false_negatives,
    }


def main():
    trials = load_trials()
    scores = rule_scores(trials)
    print(f"trials: {len(trials)}")
    print(f"rule accuracy: {scores['accuracy']:.1%}")
    print(f"false positives: {scores['false_positives']}")
    print(f"false negatives: {scores['false_negatives']}")


if __name__ == "__main__":
    main()
