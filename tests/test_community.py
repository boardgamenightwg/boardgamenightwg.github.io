"""Browser regressions against Zola-rendered Markdown, no external services needed.

Most tests remove unrelated base-template scripts to isolate the enhancement
without external services. The startup regression instead retains the complete
generated document and holds analytics pending to verify script independence.
The production Markdown is never modified; edge cases alter only served HTML.
"""

import functools
import http.server
from pathlib import Path
import re
import subprocess
import tempfile
import threading
import unittest

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass


class CommunityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.output = tempfile.TemporaryDirectory(prefix="community-tests-")
        cls.server = http.server.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            functools.partial(QuietHandler, directory=cls.output.name),
        )
        cls.url = f"http://127.0.0.1:{cls.server.server_port}"
        subprocess.run(
            [
                "zola",
                "build",
                "--force",
                "--output-dir",
                cls.output.name,
                "--base-url",
                cls.url,
            ],
            cwd=ROOT,
            check=True,
        )
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.playwright = sync_playwright().start()
        cls.browser = cls.playwright.chromium.launch()

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.playwright.stop()
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()
        cls.output.cleanup()

    def setUp(self):
        self.context = self.browser.new_context()
        self.page = self.context.new_page()
        self.page.set_default_timeout(1500)
        self.errors = []
        self.page.on("pageerror", lambda error: self.errors.append(str(error)))
        self.page.on(
            "console",
            lambda message: self.errors.append(message.text)
            if message.type == "error"
            else None,
        )
        self.use_fixture = True
        self.transform = lambda html: html

        def document(route):
            html = route.fetch().text()
            html = re.sub(
                r"<script\b[^>]*>.*?</script>",
                lambda m: m[0] if "/community.js" in m[0] else "",
                html,
                flags=re.S,
            )
            if self.use_fixture:
                fixture = (ROOT / "tests/community_fixture.html").read_text()
                html = re.sub(
                    r"(<main[^>]*>).*?(</main>)",
                    lambda m: m[1] + fixture + m[2],
                    html,
                    flags=re.S,
                )
            route.fulfill(body=self.transform(html), content_type="text/html")

        self.document_route = document
        self.page.route("**/community/", document)
        self.page.route("https://**", lambda route: route.abort())

    def tearDown(self):
        # Failed external font requests are irrelevant to isolated behavior tests.
        unexpected = [e for e in self.errors if "net::ERR_FAILED" not in e]
        self.context.close()
        self.assertEqual(unexpected, [])

    def load(self, fragment=""):
        self.page.goto(self.url + "/community/" + fragment)

    def enhanced(self, count=7):
        self.assertEqual(self.page.locator("main details").count(), count)

    def region(self, name):
        self.page.get_by_role("button", name=name, exact=True).click()

    def visible_events(self):
        return self.page.locator("main details:visible")

    def test_pending_analytics_cannot_block_community_startup(self):
        # Keep the real generated document and deferred-script ordering. Holding
        # analytics pending (not aborting it) reproduces the production failure.
        self.page.unroute("**/community/", self.document_route)
        held = []
        self.page.route("https://pls.mrkaran.dev/**", lambda route: held.append(route))
        # Icons are irrelevant here; a bounded stub keeps the real theme script
        # runnable offline without altering the document or community script.
        self.page.add_init_script("window.feather = {replace() {}};")
        try:
            self.page.goto(self.url + "/community/", wait_until="commit")
            self.page.wait_for_function("document.querySelector('main') !== null")
            self.assertTrue(held, "The real analytics request must remain pending")
            self.page.wait_for_selector("main.community-page", timeout=3000)
            self.assertNotEqual(self.page.evaluate("document.readyState"), "complete")
            self.assertGreater(self.visible_events().count(), 0)
            self.region("Bay Area")
            self.assertEqual(
                self.page.get_by_role(
                    "button", name="Bay Area", exact=True
                ).get_attribute("aria-pressed"),
                "true",
            )
            self.page.get_by_role("searchbox", name="Search events").fill(
                "no match xyz"
            )
            self.assertEqual(self.visible_events().count(), 0)
            self.page.get_by_role("button", name="Reset filters").click()
            self.assertGreater(self.visible_events().count(), 0)
        finally:
            for route in held:
                route.fulfill(body="", content_type="application/javascript")

    def test_boston_default_and_regional_controls(self):
        self.load()
        self.enhanced()
        self.assertEqual(self.visible_events().count(), 2)
        self.assertEqual(
            self.page.get_by_role("button", name="Boston", exact=True).get_attribute(
                "aria-pressed"
            ),
            "true",
        )
        for region, count in [("Bay Area", 4), ("Major Events", 1), ("All regions", 7)]:
            self.region(region)
            self.assertEqual(self.visible_events().count(), count)
            self.assertIn(
                f"{count} event", self.page.get_by_role("status").inner_text()
            )
        self.assertTrue(
            self.page.get_by_role("link", name="Submit your event below").is_visible()
        )
        self.assertTrue(self.page.locator("#submit-your-event").is_visible())
        self.assertTrue(
            self.page.get_by_role(
                "link", name="contact@boardgamenightwg.com"
            ).is_visible()
        )

    def test_every_source_heading_paragraph_link_and_id_survives(self):
        self.use_fixture = False
        self.page.route("**/community.js", lambda route: route.abort())
        self.load()
        source = self.page.locator("main").evaluate(
            "e => [...e.querySelectorAll('h2,h3,p')].map(n => n.outerHTML)"
        )
        ids = self.page.locator("main [id]").evaluate_all("es => es.map(e => e.id)")
        count = self.page.locator("main h3").count()
        self.page.unroute("**/community.js")
        self.load()
        self.enhanced(count)
        current = self.page.locator("main").evaluate(
            "e => [...e.querySelectorAll('h2,h3,p')].map(n => n.outerHTML)"
        )
        for node in source:
            self.assertEqual(current.count(node), source.count(node), node)
        for anchor in ids:
            self.assertEqual(self.page.locator(f'[id="{anchor}"]').count(), 1)
        self.assertEqual(
            self.page.get_by_role("heading", name="Robotics Talk at MIT").count(), 0
        )

    def test_search_type_and_empty_reset(self):
        self.load()
        self.region("All regions")
        search = self.page.get_by_role("searchbox", name="Search events")
        search.fill("santa clara")
        self.assertEqual(self.visible_events().count(), 1)
        search.fill("2026")
        self.page.get_by_label("Event type").select_option(label="🎲 Board Games")
        self.assertEqual(self.visible_events().count(), 1)
        search.fill("nothing matches this query")
        self.assertEqual(self.visible_events().count(), 0)
        self.assertIn("0 events", self.page.get_by_role("status").inner_text())
        self.assertTrue(
            self.page.get_by_text(
                "No events match these filters.", exact=True
            ).is_visible()
        )
        self.page.get_by_role("button", name="Reset filters").click()
        self.assertEqual(self.visible_events().count(), 7)
        self.assertEqual(search.input_value(), "")
        self.assertTrue(self.page.locator("#submit-your-event").is_visible())

    def test_native_keyboard_details_and_verbatim_schedule(self):
        self.load()
        self.region("Bay Area")
        row = self.page.locator("details").filter(
            has_text="Advanced Vision & AI Conference 2026"
        )
        summary = row.locator("summary")
        self.assertIn(
            "September 23, 2026 @ 7:30 AM – 6:30 PM; September 24 @ 7:30 AM – 4:00 PM",
            summary.inner_text(),
        )
        summary.focus()
        self.page.keyboard.press("Enter")
        self.assertTrue(row.evaluate("e => e.open"))
        self.assertTrue(row.get_by_role("link", name="More info / RSVP →").is_visible())
        self.page.keyboard.press("Space")
        self.assertFalse(row.evaluate("e => e.open"))
        self.assertIn(
            "December 6–9, 2026, program hours vary",
            self.page.locator("details")
            .filter(has_text="IEEE-RAS")
            .locator("summary")
            .inner_text(),
        )
        gated = self.page.locator("details").filter(has_text="Automated Happy Hour")
        gated.locator("summary").click()
        self.assertIn(
            "Circuit Launch, Mountain View, CA (register for the exact address)",
            gated.inner_text(),
        )
        self.region("Major Events")
        self.assertIn(
            "September 22–24, 2026, program hours vary (EDT). Opening-day registration: 7:00 AM – 5:30 PM; workshops: 8:00 AM – 5:00 PM",
            self.visible_events().locator("summary").inner_text(),
        )

    def test_existing_chapter_event_and_submission_fragments(self):
        self.load("#robot-face-earth-americas-major-robotics-events")
        self.enhanced()
        self.assertEqual(self.visible_events().count(), 1)
        self.load("#automated-happy-hour-with-rodney-brooks-robot-face-robotics")
        row = self.page.locator("details").filter(has_text="Automated Happy Hour")
        self.assertTrue(row.is_visible())
        self.assertTrue(row.evaluate("e => e.open"))
        self.region("Boston")
        self.page.get_by_role("searchbox", name="Search events").fill("no results")
        self.page.evaluate(
            "location.hash = 'ultimate-game-night-the-final-round-game-die-board-games'"
        )
        self.page.wait_for_function(
            "document.querySelector('details:has(#ultimate-game-night-the-final-round-game-die-board-games)').open"
        )
        self.assertEqual(self.visible_events().count(), 4)
        self.load("#submit-your-event")
        self.assertTrue(self.page.locator("#submit-your-event").is_visible())
        self.load("#%E0%A4%A")
        self.enhanced()

    def fragment_link(self, href):
        self.page.evaluate(
            """href => {
                document.getElementById('test-fragment-link')?.remove();
                const link = document.createElement('a');
                link.id = 'test-fragment-link';
                link.href = href;
                link.innerHTML = '<span>Revisit event</span>';
                document.body.prepend(link);
            }""",
            href,
        )
        return self.page.locator("#test-fragment-link")

    def test_current_fragment_anchor_reveals_event_after_filtering(self):
        fragment = "#automated-happy-hour-with-rodney-brooks-robot-face-robotics"
        self.load(fragment)
        row = self.page.locator("details").filter(has_text="Automated Happy Hour")
        search = self.page.get_by_role("searchbox", name="Search events")
        event_type = self.page.get_by_label("Event type")
        for href in [fragment, "/community/" + fragment, self.page.url]:
            for activation in ["click", "keyboard"]:
                with self.subTest(href=href, activation=activation):
                    self.region("Boston")
                    search.fill("no results")
                    event_type.select_option(label="🎲 Board Games")
                    row.evaluate("e => e.open = false")
                    self.assertFalse(row.is_visible())
                    link = self.fragment_link(href)
                    if activation == "click":
                        link.locator("span").click()
                    else:
                        link.focus()
                        self.page.keyboard.press("Enter")
                    self.assertTrue(row.is_visible())
                    self.assertTrue(row.evaluate("e => e.open"))
                    self.assertEqual(search.input_value(), "")
                    self.assertEqual(event_type.input_value(), "")
                    self.assertEqual(self.visible_events().count(), 4)
                    self.assertEqual(link.get_attribute("href"), href)
                    self.assertEqual(self.page.url, self.url + "/community/" + fragment)

    def test_fragment_activation_respects_native_navigation(self):
        fragment = "#automated-happy-hour-with-rodney-brooks-robot-face-robotics"
        self.load(fragment)
        self.region("Boston")
        row = self.page.locator("details").filter(has_text="Automated Happy Hour")
        cases = [
            ({}, {"ctrlKey": True}),
            ({}, {"metaKey": True}),
            ({}, {"shiftKey": True}),
            ({}, {"altKey": True}),
            ({}, {"button": 1}),
            ({}, {"cancelled": True}),
            ({"target": "_blank"}, {}),
            ({"download": "event.html"}, {}),
            ({"href": "https://example.org/community/" + fragment}, {}),
            ({"href": "/other-page/" + fragment}, {}),
            ({"href": "/community/?other-document=1" + fragment}, {}),
        ]
        for attrs, options in cases:
            with self.subTest(attrs=attrs, options=options):
                link = self.fragment_link(fragment)
                prevented = link.evaluate(
                    """(link, {attrs, options}) => {
                        for (const [name, value] of Object.entries(attrs)) {
                            link.setAttribute(name, value);
                        }
                        const event = new MouseEvent('click', {
                            bubbles: true, cancelable: true, ...options
                        });
                        if (options.cancelled) event.preventDefault();
                        let prevented;
                        // Observe the adapter, then suppress navigation in this test only.
                        window.addEventListener('click', e => {
                            prevented = e.defaultPrevented;
                            e.preventDefault();
                        }, {once: true});
                        link.dispatchEvent(event);
                        return prevented;
                    }""",
                    {"attrs": attrs, "options": options},
                )
                self.assertEqual(prevented, options.get("cancelled", False))
                self.assertFalse(row.is_visible())
                self.assertEqual(self.visible_events().count(), 2)
                self.assertEqual(
                    link.get_attribute("href"), attrs.get("href", fragment)
                )
                self.assertEqual(self.page.url, self.url + "/community/" + fragment)

    def test_current_malformed_percent_fragment_anchor_is_safe(self):
        fragment = "#%E0%A4%A"
        self.load(fragment)
        self.fragment_link(fragment).click()
        self.enhanced()
        self.assertEqual(self.visible_events().count(), 2)
        self.assertEqual(self.page.url, self.url + "/community/" + fragment)

    def test_mobile_dark_and_long_titles_no_overflow(self):
        self.load()
        self.enhanced()
        self.region("All regions")
        for width in [1200, 390, 320]:
            for dark in [False, True]:
                self.page.set_viewport_size({"width": width, "height": 900})
                self.page.evaluate(
                    "dark => document.body.dataset.theme = dark ? 'dark' : 'light'",
                    dark,
                )
                self.assertTrue(
                    self.page.evaluate(
                        "document.documentElement.scrollWidth <= innerWidth"
                    )
                )
                self.assertEqual(self.visible_events().count(), 7)
                self.assertIn(
                    "IBM Plex Sans",
                    self.page.locator("body").evaluate(
                        "e => getComputedStyle(e).fontFamily"
                    ),
                )
                self.assertEqual(
                    self.page.locator("body").evaluate(
                        "e => getComputedStyle(e).maxWidth"
                    ),
                    "720px",
                )
                self.assertTrue(
                    self.page.get_by_role(
                        "searchbox", name="Search events"
                    ).is_visible()
                )

    def test_empty_chapter_preserves_maintenance_placeholder(self):
        placeholder = (
            "No community events posted yet — be the first to submit yours below!"
        )
        self.transform = lambda html: re.sub(
            r'(<h2 id="city-sunset-boston">.*?</h2>).*?(<h2 id="bridge-at-night-sunrise-bay-area">)',
            lambda m: m[1] + f"<p><em>{placeholder}</em></p>" + m[2],
            html,
            flags=re.S,
        )
        self.load()
        self.enhanced(5)
        self.assertEqual(self.visible_events().count(), 0)
        self.assertTrue(self.page.get_by_text(placeholder, exact=True).is_visible())
        self.assertIn("0 events", self.page.get_by_role("status").inner_text())
        self.region("All regions")
        self.assertEqual(self.visible_events().count(), 5)

    def test_unrecognized_or_malformed_source_stays_unenhanced(self):
        for old, new in [
            ("<strong>Host:</strong> Test University", "Host to be confirmed"),
            ('id="city-sunset-boston">🫘🌆 Boston', 'id="future-chapter">Future Chapter'),
            (
                'id="long-title">A very long',
                'id="long-title"><a href="https://example.org">Linked</a> very long',
            ),
            (
                "<strong>When:</strong> September 11, 2026 @ 2:00 PM – 3:00 PM EDT <br>",
                "<strong>When:</strong> <br>",
            ),
        ]:
            with self.subTest(change=new):
                self.transform = lambda html: html.replace(old, new)
                self.load()
                self.assertEqual(self.page.locator("main details").count(), 0)
                self.assertEqual(self.page.locator("main h3:visible").count(), 7)
                self.assertTrue(self.page.locator("#submit-your-event").is_visible())
                self.assertEqual(self.page.get_by_role("searchbox").count(), 0)

    def test_missing_when_separator_preserves_full_original(self):
        old = "<strong>When:</strong> September 11, 2026 @ 2:00 PM – 3:00 PM EDT <br>"
        for value in ["September 11, 2026 @ 2:00 PM – 3:00 PM EDT", ""]:
            with self.subTest(when=value):
                self.transform = lambda html: html.replace(
                    old, f"<strong>When:</strong> {value} "
                )
                self.page.route("**/community.js", lambda route: route.abort())
                self.load()
                original = self.page.locator("main").inner_html()
                self.page.unroute("**/community.js")
                self.load()
                self.assertEqual(self.page.locator("main details").count(), 0)
                self.assertEqual(self.page.locator("main").inner_html(), original)
                self.assertEqual(self.page.locator("main h3:visible").count(), 7)
                self.assertEqual(self.page.get_by_role("searchbox").count(), 0)
                self.assertTrue(self.page.locator("#submit-your-event").is_visible())

    def test_initialization_exception_restores_readable_original(self):
        self.page.add_init_script(
            """
            const createElement = document.createElement.bind(document);
            document.createElement = function (name, ...args) {
                if (name === 'details') throw new Error('Simulated unsupported browser API');
                return createElement(name, ...args);
            };
        """
        )
        self.load()
        self.assertEqual(self.page.locator("main h3:visible").count(), 7)
        self.assertEqual(self.page.locator("main details").count(), 0)
        self.assertEqual(self.page.get_by_role("searchbox").count(), 0)
        self.assertTrue(self.page.locator("#submit-your-event").is_visible())

    def test_plain_text_stays_text_and_storage_is_not_required(self):
        text = '&lt;img src=x onerror="window.injected=true"&gt;'
        self.transform = lambda html: html.replace("Test University", text).replace(
            "💻 Tech &amp; Community", text
        )
        self.page.add_init_script(
            "Object.defineProperty(window, 'localStorage', {get() {throw new Error('Storage disabled')}})"
        )
        self.load()
        self.enhanced()
        self.region("All regions")
        self.page.get_by_role("searchbox", name="Search events").fill(
            '<img src=x onerror="window.injected=true">'
        )
        self.assertEqual(self.visible_events().count(), 1)
        self.assertIsNone(self.page.evaluate("window.injected"))
        self.assertEqual(self.page.locator("main img").count(), 0)
        self.assertEqual(self.page.locator("main iframe").count(), 0)
        self.assertEqual(
            self.page.get_by_label("Event type")
            .locator("option", has_text="<img")
            .count(),
            1,
        )

    def test_no_javascript_or_missing_script_leaves_readable_source(self):
        for mode in ["disabled", "blocked"]:
            with self.browser.new_context(
                java_script_enabled=mode != "disabled"
            ) as context:
                page = context.new_page()
                page.route("https://**", lambda route: route.abort())
                page.route("**/community.js", lambda route: route.abort())
                page.route("**/community/", self.document_route)
                page.goto(self.url + "/community/", wait_until="domcontentloaded")
                self.assertEqual(page.locator("main h3:visible").count(), 7)
                self.assertEqual(
                    page.locator("main a", has_text="More info / RSVP").count(), 7
                )
                self.assertTrue(
                    page.get_by_role(
                        "link", name="contact@boardgamenightwg.com"
                    ).is_visible()
                )
                self.assertEqual(page.get_by_role("searchbox").count(), 0)


if __name__ == "__main__":
    unittest.main()
