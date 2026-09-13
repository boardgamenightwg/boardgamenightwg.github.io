"""Homepage contracts on real Zola builds (including source-only fixture edits)."""

import functools
import http.server
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import unittest

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]


def event(title, date, region_note=""):
    return f"""### {title} <span class="badge badge-robotics">🤖 Robotics</span>
**When:** {date} \\
**Where:** Fixture Hall, 12 Example St \\
**Host:** Fixture Organizer \\
Full description with **important access** and {region_note}.

[More info / RSVP →](https://example.org/register)

"""


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


class HomepageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="homepage-tests-")
        cls.root = Path(cls.temp.name)
        cls.server = http.server.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            functools.partial(QuietHandler, directory=cls.temp.name),
        )
        cls.url = f"http://127.0.0.1:{cls.server.server_port}"
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.fixture = cls.root / "source"
        shutil.copytree(
            ROOT,
            cls.fixture,
            ignore=shutil.ignore_patterns(".git", "public", "__pycache__", ".venv*"),
        )
        # The only fixture edits are canonical Markdown. No build preprocessor.
        (cls.fixture / "content/boston.md").write_text(
            """+++
title = "Boston"
+++
## Where
**Fixture Venue** \\
**Not the venue: street address**

## When
September 24, 2026 @ 6:00 PM – 9:00 PM

## RSVP
<script>window.sourceExecuted = true</script>
<iframe src="https://source.invalid/frame"></iframe>
"""
        )
        (cls.fixture / "content/bayarea.md").write_text(
            """+++
title = "Bay Area"
+++
## When
Date to be announced
"""
        )
        fixture_events = (
            '+++\ntitle = "Community"\ntemplate = "community.html"\n+++\n\n'
        )
        fixture_events += "## 🫘🌆 Boston\n\n"
        fixture_events += event("Expired fixture", "September 1, 2026")
        fixture_events += event(
            "Future fixture",
            "September 26, 2026",
            '<a href="javascript:alert(1)" onclick="alert(2)">unsafe link</a> '
            '<a id="bookmark">named anchor</a> '
            '<a href="data:text/html,unsafe">data link</a> '
            '<a href="vbscript:msgbox(1)">vbscript link</a> '
            '<a href="details/registration">relative link</a> '
            '<a href="">self link</a> '
            '<img src="https://source.invalid/image" onerror="alert(3)">',
        )
        fixture_events += event("Unknown fixture", "Date TBD")
        fixture_events += "## 🌉🌅 Bay Area\n\n"
        fixture_events += event(
            "Ongoing fixture", "September 23, 2026 @ 7 AM; September 24 @ 8 AM"
        )
        fixture_events += event("Spare fixture", "September 28, 2026")
        fixture_events += "## 🤖🌎 Major Robotics Events\n\n"
        fixture_events += event(
            "Range fixture", "September 22–24, 2026, program hours vary"
        )
        fixture_events += '<script>window.sourceExecuted = true</script>\n<iframe src="https://source.invalid/community"></iframe>\n'
        fixture_events += "## Submit Your Event\n\nNot an event\n"
        (cls.fixture / "content/community.md").write_text(fixture_events)
        for source, output in [(ROOT, "actual"), (cls.fixture, "fixture")]:
            subprocess.run(
                [
                    "zola",
                    "build",
                    "--output-dir",
                    str(cls.root / output),
                    "--base-url",
                    cls.url + "/" + output,
                ],
                cwd=source,
                check=True,
                capture_output=True,
                text=True,
            )
        cls.playwright = sync_playwright().start()
        cls.browser = cls.playwright.chromium.launch()

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.playwright.stop()
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()
        cls.temp.cleanup()

    def setUp(self):
        self.context = self.browser.new_context(
            viewport={"width": 1200, "height": 1100}, timezone_id="Asia/Tokyo"
        )
        self.page = self.context.new_page()
        self.page.set_default_timeout(2500)
        self.errors = []
        self.requests = []
        self.page.on("pageerror", lambda error: self.errors.append(str(error)))
        self.page.on("request", lambda request: self.requests.append(request.url))
        self.page.route("https://**", lambda route: route.abort())
        # Keep inherited theme runnable offline; no document/script rewriting.
        self.page.add_init_script("window.feather = {replace() {}};")
        self.page.clock.set_fixed_time("2026-09-24T18:00:00Z")

    def tearDown(self):
        self.context.close()
        self.assertEqual(self.errors, [])

    def load(self, source="fixture", enhanced=True):
        self.page.goto(self.url + "/" + source + "/", wait_until="commit")
        if enhanced:
            self.page.wait_for_selector("main")
            try:
                self.page.wait_for_selector("main[data-homepage-ready]")
            except Exception:
                self.fail(
                    "Homepage enhancement did not render: "
                    + self.page.locator("main").inner_text()
                )
        else:
            self.page.wait_for_selector("#club-title")

    def test_model_contracts_in_unittest_discovery(self):
        result = subprocess.run(
            ["node", "--test", "tests/homepage-model.test.mjs"],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_approved_structure_and_source_only_build_propagation(self):
        self.load()
        self.assertEqual(
            self.page.locator("main > section > h2").all_text_contents(),
            ["Next club game nights", "Around the community"],
        )
        clubs = self.page.locator(".club-row")
        self.assertEqual(clubs.count(), 2)
        self.assertIn(
            "September 24, 2026 @ 6:00 PM – 9:00 PM", clubs.first.inner_text()
        )
        self.assertIn("Fixture Venue", clubs.first.inner_text())
        self.assertNotIn("street address", clubs.first.inner_text())
        self.assertIn("Details / RSVP", clubs.first.inner_text())
        self.assertIn("Check the chapter for the latest date", clubs.last.inner_text())
        self.assertNotIn("coming soon", clubs.last.inner_text())
        self.assertEqual(
            self.page.locator(".event-title").all_text_contents(),
            ["Range fixture", "Ongoing fixture", "Future fixture"],
        )
        self.assertEqual(
            self.page.locator(".date-tile b").all_text_contents(), ["22", "23", "26"]
        )
        self.assertEqual(self.page.locator("main iframe").count(), 0)
        self.assertNotIn("DESIGN PREVIEW", self.page.locator("body").inner_text())
        self.assertTrue(
            self.page.get_by_role("link", name="Start with the FAQ").is_visible()
        )
        self.assertTrue(
            self.page.get_by_role("link", name="Established in 2012").is_visible()
        )

    def test_keyboard_disclosures_preserve_schedule_metadata_and_safe_links(self):
        self.load()
        row = self.page.locator(".preview-event").filter(has_text="Ongoing fixture")
        self.assertEqual(
            row.locator(".schedule").inner_text(),
            "September 23, 2026 @ 7 AM; September 24 @ 8 AM",
        )
        summary = row.locator("summary")
        summary.focus()
        self.page.keyboard.press("Enter")
        self.assertTrue(row.evaluate("e => e.open"))
        for text in [
            "When:",
            "Where:",
            "Host:",
            "Fixture Hall, 12 Example St",
            "Fixture Organizer",
            "Full description with important access",
        ]:
            self.assertIn(text, row.locator(".event-detail").inner_text())
        self.assertEqual(
            row.get_by_role("link", name="More info / RSVP").get_attribute("href"),
            "https://example.org/register",
        )
        self.assertIn(
            "/community/#ongoing-fixture",
            row.get_by_role("link", name="View on our community page").get_attribute(
                "href"
            ),
        )
        self.page.keyboard.press("Space")
        self.assertFalse(row.evaluate("e => e.open"))
        future = self.page.locator(".preview-event").filter(has_text="Future fixture")
        for text in ["named anchor", "unsafe link", "data link", "vbscript link"]:
            anchor = future.locator("a").filter(has_text=text)
            self.assertEqual(anchor.count(), 1)
            self.assertIsNone(anchor.get_attribute("href"), text)
        for text, suffix in [
            ("relative link", "details/registration"),
            ("self link", ""),
        ]:
            self.assertEqual(
                future.locator("a").filter(has_text=text).get_attribute("href"),
                self.url + "/fixture/community/" + suffix,
            )
        self.assertEqual(
            self.page.locator(
                'main [onclick], main [onerror], main a[href^="javascript:"], '
                'main a[href^="data:"], main a[href^="vbscript:"], main img, main iframe'
            ).count(),
            0,
        )
        self.assertIsNone(self.page.evaluate("window.sourceExecuted"))
        self.assertFalse(
            any(
                "source.invalid" in url or "embed.lu.ma" in url for url in self.requests
            )
        )
        self.assertEqual(self.page.locator("main time").count(), 0)

    def test_actual_sources_not_snapshot_counts_or_titles(self):
        self.load("actual")
        source = self.page.locator("#homepage-community-source").evaluate(
            't => [...t.content.querySelectorAll("h3")].map(e => e.textContent.trim())'
        )
        titles = self.page.locator(".event-title").all_text_contents()
        self.assertLessEqual(len(titles), 3)
        for title in titles:
            self.assertTrue(any(heading.startswith(title) for heading in source))
        for name, path in [("Boston", "/boston/"), ("Bay Area", "/bayarea/")]:
            self.assertTrue(
                self.page.locator("main")
                .get_by_role("link", name=name, exact=True)
                .get_attribute("href")
                .endswith(path)
            )
        self.assertFalse(
            any("embed.lu.ma" in url or "maps.google" in url for url in self.requests)
        )

    def test_expired_sources_never_claim_upcoming_and_directory_stays(self):
        self.page.clock.set_fixed_time("2030-01-01T18:00:00Z")
        self.load()
        self.assertEqual(self.page.locator(".preview-event").count(), 0)
        self.assertIn(
            "Next date coming soon", self.page.locator(".club-row").first.inner_text()
        )
        self.assertTrue(
            self.page.get_by_role(
                "link", name="Browse all community events"
            ).is_visible()
        )
        self.assertIn(
            "See the community directory for more dates",
            self.page.locator("main").inner_text(),
        )

    def test_unsupported_chapter_markup_keeps_honest_chapter_signpost(self):
        def document(route):
            html = (
                route.fetch()
                .text()
                .replace(
                    "<p>Date to be announced</p>", "<div>Date to be announced</div>"
                )
            )
            route.fulfill(body=html, content_type="text/html")

        self.page.route("**/fixture/", document)
        self.load()
        self.assertIn(
            "Check the chapter for the latest date",
            self.page.locator(".club-row").last.inner_text(),
        )
        self.assertNotIn(
            "Next date coming soon", self.page.locator(".club-row").last.inner_text()
        )

    def test_nojs_and_blocked_module_fallback_keeps_navigation_without_false_empty_claim(
        self,
    ):
        for disabled in [True, False]:
            with self.subTest(disabled=disabled):
                context = self.browser.new_context(java_script_enabled=not disabled)
                page = context.new_page()
                page.route("https://**", lambda route: route.abort())
                page.route("**/scripts/homepage*.mjs", lambda route: route.abort())
                page.goto(self.url + "/actual/", wait_until="commit")
                page.wait_for_selector("main")
                text = page.locator("main").inner_text()
                self.assertIn("Next club game nights", text)
                for title in [
                    "Boston",
                    "Bay Area",
                    "Browse all community events",
                    "Established in 2012",
                ]:
                    self.assertTrue(
                        page.locator("main")
                        .get_by_role("link", name=title)
                        .is_visible()
                    )
                self.assertNotIn("Next date coming soon", text)
                self.assertNotIn("No upcoming", text)
                self.assertEqual(page.locator("main iframe").count(), 0)
                context.close()

    def test_real_pending_analytics_does_not_block_enhancement(self):
        held = []
        self.page.route("https://pls.mrkaran.dev/**", lambda route: held.append(route))
        try:
            self.load("fixture")
            self.assertTrue(held, "Analytics must really remain pending")
            self.assertNotEqual(self.page.evaluate("document.readyState"), "complete")
            self.assertGreater(self.page.locator(".preview-event").count(), 0)
            self.page.locator(".preview-event summary").first.click()
            self.assertTrue(
                self.page.locator(".preview-event").first.evaluate("e => e.open")
            )
        finally:
            for route in held:
                route.fulfill(body="", content_type="application/javascript")

    def test_mobile_and_theme_preserve_compact_rows_and_focus(self):
        self.load()
        self.assertEqual(
            self.page.locator("body").evaluate("e => getComputedStyle(e).maxWidth"),
            "720px",
        )
        tile = self.page.locator(".date-tile").first
        self.assertAlmostEqual(tile.bounding_box()["width"], 44, delta=1)
        self.assertLess(
            self.page.locator(".preview-event").first.bounding_box()["height"], 150
        )
        for width in [320, 390, 1200]:
            self.page.set_viewport_size({"width": width, "height": 1100})
            for theme in ["dark", "light"]:
                self.page.get_by_role("button", name="Toggle Dark Mode").click()
                self.assertEqual(
                    self.page.locator("body").get_attribute("data-theme"), theme
                )
                self.assertTrue(
                    self.page.evaluate(
                        "document.documentElement.scrollWidth <= innerWidth"
                    )
                )
                summary = self.page.locator(".preview-event summary").first
                self.page.keyboard.press("Tab")
                summary.focus()
                self.assertNotEqual(
                    summary.evaluate("e => getComputedStyle(e).outlineStyle"), "none"
                )
                self.assertEqual(self.page.locator(".preview-event").count(), 3)
                self.assertTrue(
                    self.page.get_by_role(
                        "link", name="Browse all community events"
                    ).is_visible()
                )


if __name__ == "__main__":
    unittest.main()
