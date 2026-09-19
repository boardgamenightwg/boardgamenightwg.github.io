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

    def test_explicitly_unlisted_careers(self):
        data = fixture()
        data["companies"][0]["careers_url"] = None
        self.assertEqual(validate(data), [])
        del data["companies"][0]["careers_url"]
        self.assertTrue(any("missing fields" in e for e in validate(data)))

    def test_careers_null_is_not_an_empty_or_invalid_url(self):
        for value in ["", " ", False, [], {}]:
            data = fixture()
            data["companies"][0]["careers_url"] = value
            self.assertTrue(any("bad careers_url" in e for e in validate(data)))

    def test_optional_verified_location(self):
        for precision in ["city", "address"]:
            data = fixture()
            data["companies"][0]["location"] = self.location(precision=precision)
            self.assertEqual(validate(data), [])

    @staticmethod
    def location(**changes):
        return dict(
            dict(
                label="Waltham, MA",
                lat=42.3765,
                lon=-71.2356,
                precision="city",
                source_url="https://example.org/contact",
                verified="2026-09-10",
            ),
            **changes,
        )

    def test_rejects_invalid_locations(self):
        cases = [None, [], {}, self.location(extra="no")]
        for field, values in {
            "lat": [True, False, "42", None, [], float("nan"), float("inf"), -91, 91],
            "lon": [True, "-71", float("nan"), float("-inf"), -181, 181],
            "precision": ["roof", "", None, []],
            "label": ["", "  ", None, 12, "x" * 201],
            "source_url": [
                "http://example.org",
                "javascript:alert(1)",
                "https://user:pass@example.org",
                "not a URL",
            ],
            "verified": ["2026-02-30", "20260910", "2030-01-01", None],
        }.items():
            cases.extend(self.location(**{field: value}) for value in values)
        for field in self.location():
            location = self.location()
            del location[field]
            cases.append(location)
        for location in cases:
            with self.subTest(location=location):
                data = fixture()
                data["companies"][0]["location"] = location
                self.assertTrue(any("location" in e for e in validate(data)))

    def test_coordinate_boundaries_are_valid(self):
        for lat, lon in [(-90, -180), (90, 180), (0, 0)]:
            data = fixture()
            data["companies"][0]["location"] = self.location(lat=lat, lon=lon)
            self.assertEqual(validate(data), [])

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
