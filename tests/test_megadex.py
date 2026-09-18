"""Stdlib contract tests for the Megadex validator and data."""

import copy
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/validate_megadex.py"
DATA = ROOT / "static/data/megadex.json"

spec = importlib.util.spec_from_file_location("validate_megadex", SCRIPT)
validate_megadex = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validate_megadex)
validate = validate_megadex.validate


def fixture():
    company = dict(
        id="acme",
        name="Acme Robotics",
        region="boston",
        website="https://example.org/",
        careers_url="https://example.org/careers",
        summary="A robotics company.",
        news=[
            dict(
                date="2026-09-01",
                headline="Acme raises Series B",
                url="https://example.org/news/series-b",
            ),
            dict(
                date="2026-08-01",
                headline="Acme opens Boston office",
                url="https://example.org/news/office",
            ),
        ],
        last_verified="2026-09-10",
    )
    return dict(
        version=1,
        regions={"boston": "Boston", "bay": "Bay Area"},
        companies=[company, dict(company, id="other", name="Other Co")],
    )


class ValidatorTests(unittest.TestCase):
    def test_fixture_is_valid(self):
        self.assertEqual(validate(fixture()), [])

    def test_real_data_is_valid(self):
        data = json.loads(DATA.read_text(encoding="utf-8"))
        self.assertEqual(validate(data), [])

    def test_rejects_extra_and_missing_fields(self):
        data = fixture()
        data["companies"][0]["nickname"] = "Ace"
        del data["companies"][1]["summary"]
        errors = validate(data)
        self.assertTrue(any("unexpected fields" in e for e in errors))
        self.assertTrue(any("missing fields" in e for e in errors))

    def test_rejects_unknown_region(self):
        data = fixture()
        data["companies"][0]["region"] = "chicago"
        self.assertTrue(any("unknown region" in e for e in validate(data)))

    def test_rejects_bad_urls(self):
        for bad in [
            "javascript:alert(1)",
            "ftp://example.org",
            "not a url",
            "https://user:***@example.org/",
        ]:
            data = fixture()
            data["companies"][0]["careers_url"] = bad
            self.assertTrue(
                any("bad careers_url" in e for e in validate(data)),
                f"expected rejection for {bad!r}",
            )

    def test_rejects_future_last_verified(self):
        data = fixture()
        data["companies"][0]["last_verified"] = "2030-01-01"
        self.assertTrue(any("bad last_verified" in e for e in validate(data)))

    def test_rejects_bad_news(self):
        data = fixture()
        data["companies"][0]["news"][0]["date"] = "09/01/2026"
        self.assertTrue(any("bad date" in e for e in validate(data)))

        data = fixture()
        data["companies"][0]["news"] = list(reversed(data["companies"][0]["news"]))
        self.assertTrue(any("newest-first" in e for e in validate(data)))

        data = fixture()
        data["companies"][0]["news"][1]["url"] = data["companies"][0]["news"][0]["url"]
        self.assertTrue(any("duplicate news url" in e for e in validate(data)))

    def test_rejects_duplicate_company_ids(self):
        data = fixture()
        data["companies"][1]["id"] = data["companies"][0]["id"]
        self.assertTrue(any("duplicate id" in e for e in validate(data)))

    def test_validate_never_mutates(self):
        data = fixture()
        before = copy.deepcopy(data)
        validate(data)
        self.assertEqual(data, before)


if __name__ == "__main__":
    unittest.main()
