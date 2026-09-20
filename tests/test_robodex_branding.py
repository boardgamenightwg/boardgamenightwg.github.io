"""Public directory resources use the Robodex name end to end."""

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RobodexBrandingTests(unittest.TestCase):
    def test_resource_paths(self):
        resources = (
            "content/robodex.md",
            "templates/robodex.html",
            "static/data/robodex.json",
            "static/scripts/robodex.mjs",
            "static/styles/robodex.css",
            "scripts/validate_robodex.py",
            "tests/test_robodex.py",
            "tests/robodex_smoke.py",
            ".github/workflows/robodex.yaml",
            "docs/robodex.md",
            "docs/robodex-amazon-sources.md",
            "docs/robodex-company-sources.md",
            "docs/robodex-hosts.md",
            "docs/robodex-locations.md",
            "docs/robodex-regional-sources.md",
        )
        for resource in resources:
            with self.subTest(resource=resource):
                self.assertTrue((ROOT / resource).is_file(), resource)

    def test_page_uses_directory_template(self):
        self.assertIn(
            'template = "robodex.html"', (ROOT / "content/robodex.md").read_text()
        )


if __name__ == "__main__":
    unittest.main()
