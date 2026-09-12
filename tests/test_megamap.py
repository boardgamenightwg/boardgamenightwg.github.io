"""Stdlib contract tests; no site or browser dependencies."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/validate_megamap.py"


def fixture():
    node = dict(
        id="one",
        name="One",
        region="boston",
        category="robotics",
        summary="A community.",
        participation="Check announcements.",
        sources=[dict(label="About", url="https://example.org/")],
        last_verified="2026-09-11",
        status="reviewed",
    )
    other = dict(node, id="two", name="Two")
    return dict(
        version=1,
        regions={"boston": "Boston"},
        categories={"robotics": "Robotics"},
        nodes=[node, other],
        edges=[
            dict(
                id="one-shares-two",
                source="one",
                target="two",
                type="shares_events_from",
                description="Lists a specific event, not a partnership.",
                sources=node["sources"],
                last_verified="2026-09-11",
                status="reviewed",
            )
        ],
    )


class ValidatorTests(unittest.TestCase):
    def validate(self, data):
        self.assertTrue(SCRIPT.is_file(), "Missing stdlib Megamap validator")
        spec = importlib.util.spec_from_file_location("validator", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.validate(data)

    def test_valid_data(self):
        self.assertEqual(self.validate(fixture()), [])

    def test_invalid_records(self):
        mutations = [
            lambda d: d["nodes"].append(copy.deepcopy(d["nodes"][0])),
            lambda d: d["nodes"][0].update(id="Bad ID"),
            lambda d: d["nodes"][0].update(region="unknown"),
            lambda d: d["nodes"][0].update(category="unknown"),
            lambda d: d["nodes"][0].update(summary=" "),
            lambda d: d["nodes"][0].update(participation=42),
            lambda d: d["nodes"][0].update(status="verified"),
            lambda d: d["nodes"][0].update(last_verified="2026-02-30"),
            lambda d: d["nodes"][0].update(last_verified="20260911"),
            lambda d: d["nodes"][0].update(last_verified=None),
            lambda d: d["nodes"][0].update(sources=[]),
            lambda d: d["nodes"][0].update(unexpected="typo"),
            lambda d: d["nodes"][0]["sources"][0].update(url="javascript:alert(1)"),
            lambda d: d["nodes"][0]["sources"][0].update(url="https://"),
            lambda d: d["nodes"][0]["sources"][0].update(url="//example.org/"),
            lambda d: d["nodes"][0]["sources"][0].update(
                url="https://user:pass@example.org"
            ),
            lambda d: d["nodes"][0]["sources"][0].update(
                url="https://example.org/\nfoo"
            ),
            lambda d: d["edges"][0].update(target="missing"),
            lambda d: d["edges"][0].update(target="one"),
            lambda d: d["edges"][0].update(type="partners_with"),
            lambda d: d["edges"][0].update(description=""),
            lambda d: d["edges"][0].update(sources=[]),
            lambda d: d["edges"][0].update(last_verified="yesterday"),
            lambda d: d["edges"][0].update(status="seed"),
            lambda d: d["edges"].append(dict(d["edges"][0], id="duplicate")),
            lambda d: d.update(version=2),
            lambda d: d.update(nodes="invalid"),
            lambda d: d.update(regions={}),
            lambda d: d["categories"].update(
                all="Conflicts with all-categories filter"
            ),
            lambda d: d["nodes"][0]["sources"][0].update(url="https://<invalid>/"),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutations.index(mutate)):
                data = copy.deepcopy(fixture())
                mutate(data)
                self.assertTrue(self.validate(data))

    def test_symmetric_duplicates_but_directed_reverse_allowed(self):
        data = fixture()
        data["edges"].append(
            dict(data["edges"][0], id="reverse", source="two", target="one")
        )
        self.assertEqual(self.validate(data), [])
        for edge in data["edges"]:
            edge["type"] = "runs_activities_with"
        self.assertTrue(self.validate(data))

    def test_seed_is_transparently_unverified_and_has_no_edges(self):
        data = fixture()
        data["nodes"][0].update(status="seed", sources=[], last_verified=None)
        self.assertTrue(self.validate(data))
        data["edges"] = []
        self.assertEqual(self.validate(data), [])

    def test_cli_rejects_duplicate_json_keys_and_invalid_json(self):
        self.assertTrue(SCRIPT.is_file(), "Missing validator CLI")
        for text in ['{"version":1,"version":1}', "{"]:
            with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as handle:
                handle.write(text)
                handle.flush()
                result = subprocess.run(
                    [sys.executable, str(SCRIPT), handle.name],
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 1)
                self.assertNotIn("Traceback", result.stderr)

    def test_approved_dataset(self):
        path = ROOT / "static/data/megamap.json"
        self.assertTrue(path.is_file(), "Missing canonical dataset")
        data = json.loads(path.read_text())
        self.assertEqual(self.validate(data), [])
        # Do not freeze counts or statuses: reviewed data-only PRs must be able to grow the map.
        self.assertTrue(data["nodes"])


if __name__ == "__main__":
    unittest.main()
