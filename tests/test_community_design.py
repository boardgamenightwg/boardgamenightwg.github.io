"""Chapter-first design regressions on actual Zola content, plus safe date edges."""

import unittest

import test_community as community


class CommunityDesignTests(unittest.TestCase):
    # Reuse the browser harness, not its test methods (no inherited duplicate suite).
    setUpClass = classmethod(community.CommunityTests.setUpClass.__func__)
    tearDownClass = classmethod(community.CommunityTests.tearDownClass.__func__)
    tearDown = community.CommunityTests.tearDown
    load = community.CommunityTests.load
    region = community.CommunityTests.region
    visible_events = community.CommunityTests.visible_events

    def setUp(self):
        community.CommunityTests.setUp(self)
        self.use_fixture = False
        self.page.set_viewport_size({"width": 1200, "height": 1050})

    def test_chapter_heading_and_description_precede_filters(self):
        self.load()
        heading = self.page.locator(".community-chapter-head h2")
        self.assertEqual(
            heading.count(), 1, "Chapter-first heading missing above filters"
        )
        self.assertEqual(heading.inner_text(), "Boston")
        description = self.page.locator(".community-chapter-head p")
        self.assertEqual(
            description.inner_text(), "Talks, demos, and game nights around Boston."
        )
        search = self.page.get_by_role("searchbox", name="Search events")
        self.assertLess(heading.bounding_box()["y"], description.bounding_box()["y"])
        self.assertLess(description.bounding_box()["y"], search.bounding_box()["y"])
        self.assertFalse(
            self.page.get_by_role("button", name="Reset filters").is_visible()
        )
        self.region("Bay Area")
        self.assertEqual(heading.inner_text(), "Bay Area")
        self.assertEqual(
            description.inner_text(), "Talks, demos, and game nights around Bay Area."
        )
        self.region("Major Events")
        self.assertEqual(
            description.inner_text(), "Robotics gatherings worth the trip."
        )
        self.region("All regions")
        self.assertEqual(description.inner_text(), "Explore the whole community.")

    def test_sidebar_has_decorative_chapter_emojis_and_club_links(self):
        self.load()
        for name, emoji in [
            ("Boston", "🫘🌆"),
            ("Bay Area", "🌉🌅"),
            ("Major Events", "🤖🌎"),
        ]:
            button = self.page.get_by_role("button", name=name, exact=True)
            decoration = button.locator("[aria-hidden=true]")
            self.assertEqual(decoration.count(), 1, "Region emoji must be decorative")
            self.assertEqual(decoration.inner_text(), emoji)
        sidebar = self.page.locator(".community-sidebar")
        self.assertIn("Community-organized events.", sidebar.inner_text())
        self.assertIn("Club game nights stay on the", sidebar.inner_text())
        for name, path in [("Boston", "/boston/"), ("Bay Area", "/bayarea/")]:
            self.assertTrue(
                sidebar.get_by_role("link", name=name, exact=True)
                .get_attribute("href")
                .endswith(path)
            )

    def test_source_events_are_grouped_once_by_month_and_year(self):
        self.load()
        headings = self.page.locator(".community-month:visible > h2")
        self.assertEqual(
            headings.all_text_contents(),
            ["September 2026", "October 2026", "November 2026"],
        )
        self.assertEqual(
            self.page.locator(".community-month:visible").evaluate_all(
                "es => es.map(e => e.querySelectorAll('details:not([hidden])').length)"
            ),
            [1, 2, 1],
        )
        self.assertEqual(
            headings.first.evaluate("e => getComputedStyle(e).borderBottomStyle"),
            "dashed",
        )
        self.region("All regions")
        self.assertEqual(
            headings.all_text_contents(),
            ["September 2026", "October 2026", "November 2026", "December 2026"],
        )
        self.assertEqual(self.visible_events().count(), 10)
        self.assertEqual(
            self.page.locator(".community-month:visible").evaluate_all(
                "es => es.map(e => e.querySelectorAll('details:not([hidden])').length)"
            ),
            [5, 3, 1, 1],
        )
        self.page.get_by_role("searchbox", name="Search events").fill("Minds in Motion")
        self.assertEqual(headings.all_text_contents(), ["October 2026"])
        self.page.get_by_role("searchbox", name="Search events").fill(
            "no match anywhere"
        )
        self.assertEqual(headings.count(), 0)
        self.assertTrue(
            self.page.get_by_role("button", name="Reset filters").is_visible()
        )

    def test_compact_date_column_and_eyebrow_above_unchanged_title(self):
        self.load()
        row = self.visible_events().first
        tile = row.locator(".community-date-tile")
        self.assertEqual(tile.count(), 1, "Approved month/day column missing")
        self.assertEqual(tile.locator("span").inner_text(), "Sep")
        self.assertEqual(tile.locator("b").inner_text(), "26")
        self.assertEqual(tile.get_attribute("aria-hidden"), "true")
        title = row.locator("h3")
        eyebrow = row.locator(".community-eyebrow")
        self.assertIn("Boston", eyebrow.inner_text())
        self.assertIn("Robotics", eyebrow.inner_text())
        self.assertLess(eyebrow.bounding_box()["y"], title.bounding_box()["y"])
        self.assertLess(tile.bounding_box()["x"], title.bounding_box()["x"])
        self.assertAlmostEqual(tile.bounding_box()["width"], 44, delta=1)
        self.assertEqual(
            tile.locator("b").evaluate("e => getComputedStyle(e).fontSize"), "26px"
        )
        self.assertFalse(title.locator(".badge").is_visible())
        self.assertEqual(row.locator("summary .badge:visible").count(), 1)
        self.assertLess(row.bounding_box()["height"], 140)
        schedule = "September 26, 2026 @ 10:45 AM – 4:00 PM"
        self.assertEqual(row.locator(".community-date").inner_text(), schedule)
        self.assertFalse(row.evaluate("e => e.open"))
        row.locator("summary").click()
        self.assertTrue(row.get_by_role("link", name="More info / RSVP →").is_visible())

    def test_ranges_and_full_multi_day_schedules_keep_verbatim_text(self):
        self.load()
        self.region("All regions")
        for title, month, day, schedule in [
            (
                "Advanced Vision & AI Conference 2026",
                "Sep",
                "23",
                "September 23, 2026 @ 7:30 AM – 6:30 PM; September 24 @ 7:30 AM – 4:00 PM",
            ),
            (
                "2026 IEEE-RAS International Conference on Humanoid Robots",
                "Dec",
                "6",
                "December 6–9, 2026, program hours vary",
            ),
            (
                "ROSCon Global 2026",
                "Sep",
                "22",
                "September 22–24, 2026, program hours vary (EDT). Opening-day registration: 7:00 AM – 5:30 PM; workshops: 8:00 AM – 5:00 PM",
            ),
        ]:
            with self.subTest(title=title):
                row = self.page.locator("details").filter(has_text=title)
                self.assertEqual(row.locator(".community-date-tile").count(), 1)
                self.assertEqual(
                    row.locator(".community-date-tile span").inner_text(), month
                )
                self.assertEqual(
                    row.locator(".community-date-tile b").inner_text(), day
                )
                self.assertEqual(row.locator(".community-date").inner_text(), schedule)
                self.assertEqual(
                    row.locator("time").count(), 0, "Do not infer a machine timestamp"
                )

    def test_unrecognized_or_invalid_dates_use_honest_fallback(self):
        original = "September 26, 2026 @ 10:45 AM – 4:00 PM"
        for schedule in [
            "Date to be announced",
            "09/26/26, time unknown",
            "February 30, 2026, hours vary",
            "September 26, 20260",
            "September 28–24, 2026",
            "February 29, 2027",
        ]:
            with self.subTest(schedule=schedule):
                self.transform = lambda html: html.replace(original, schedule)
                self.load()
                row = self.page.locator("details").filter(has_text="RoboBoston:")
                group = row.locator("..")
                self.assertEqual(group.locator(":scope > h2").count(), 1)
                self.assertEqual(
                    group.locator(":scope > h2").inner_text(), "Other dates"
                )
                self.assertEqual(row.locator(".community-date-tile").count(), 0)
                self.assertEqual(row.locator(".community-date").inner_text(), schedule)
                self.assertEqual(self.visible_events().count(), 4)

    def test_year_is_taken_from_source_not_fixed_to_mockup(self):
        self.transform = lambda html: html.replace(
            "September 26, 2026", "February 29, 2028"
        )
        self.load()
        row = self.page.locator("details").filter(has_text="RoboBoston:")
        self.assertEqual(row.locator(".community-date-tile").count(), 1)
        self.assertEqual(row.locator(".community-date-tile span").inner_text(), "Feb")
        self.assertEqual(row.locator(".community-date-tile b").inner_text(), "29")
        self.assertEqual(
            row.locator("..").locator(":scope > h2").inner_text(), "February 2028"
        )


if __name__ == "__main__":
    unittest.main()
