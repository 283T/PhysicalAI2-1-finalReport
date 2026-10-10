"""Evaluate image reading against fixtures that are never sent to the model."""

import json

from local_vision import MODEL
from project_paths import ROOT
from read_drawing import read_drawing


def main():
    results = []
    keys = ("drawing_id", "socket", "component", "harness_id", "plug_id", "x_mm", "y_mm", "yaw_deg")
    for variant in ("a", "b"):
        expected = json.loads((ROOT / f"examples/task_{variant}.json").read_text())
        output = ROOT / f"outputs/ai_task_{variant}.json"
        try:
            actual = read_drawing(ROOT / f"drawings/assembly_{variant}.png", output)
            matches = {key: actual[key] == expected[key] for key in keys}
            audit = json.loads(output.with_suffix(".audit.json").read_text())
            results.append(
                {
                    "variant": variant.upper(),
                    "success": all(matches.values()),
                    "matches": matches,
                    "extracted_task": actual,
                    "elapsed_seconds": audit["elapsed_seconds"],
                }
            )
        except Exception as error:
            results.append({"variant": variant.upper(), "success": False, "error": str(error)})
    report = {
        "model": MODEL,
        "cases": results,
        "passed": sum(case["success"] for case in results),
        "total": len(results),
    }
    (ROOT / "outputs/drawing_evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if report["passed"] != report["total"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
