"""Optional browser smoke: build to public/ with localhost base URL first."""
import functools
import http.server
import json
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ROOT / "public"


def main():
    assert (PUBLIC / "megamap/index.html").is_file(), "Missing built Megamap route"
    data = json.loads((ROOT / "static/data/megamap.json").read_text())
    region_nodes = lambda region: [n for n in data["nodes"] if n["region"] == region]

    def edge_count(region):
        ids = {n["id"] for n in region_nodes(region)}
        return sum(e["source"] in ids and e["target"] in ids for e in data["edges"])

    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=str(PUBLIC)
    )
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 8766), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch()
            context = browser.new_context(viewport={"width": 1440, "height": 1000})
            page = context.new_page()
            errors = []
            console_errors = []
            # Existing site analytics is unrelated and may hang offline/CI.
            context.route("https://pls.mrkaran.dev/**", lambda route: route.abort())
            page.on(
                "console",
                lambda msg: console_errors.append(msg.text)
                if msg.type == "error"
                and not msg.location.get("url", "").startswith(
                    "https://pls.mrkaran.dev/"
                )
                else None,
            )
            page.on("pageerror", lambda error: errors.append(str(error)))
            # A hanging inherited deferred analytics script must not gate enhancement.
            hung = context.new_page()
            held_routes = []
            hung.route(
                "https://pls.mrkaran.dev/**", lambda route: held_routes.append(route)
            )
            hung.goto("http://127.0.0.1:8766/megamap/", wait_until="commit")
            try:
                expect(hung.locator("#mm-app")).to_be_visible(timeout=5000)
                assert (
                    held_routes
                ), "Analytics must actually be held pending for this regression"
                assert hung.evaluate("document.readyState") != "complete"
            finally:
                for route in held_routes:
                    route.abort()
                hung.close()
            page.goto("http://127.0.0.1:8766/megamap/", wait_until="domcontentloaded")
            expect(page.locator("#mm-app")).to_be_visible()
            expect(page.locator('meta[name="robots"]')).to_have_attribute(
                "content", "noindex,follow"
            )
            assert "megamap" not in (PUBLIC / "sitemap.xml").read_text()
            assert page.locator('header a[href*="megamap"]').count() == 0
            expect(page.locator("#mm-nodes button:visible")).to_have_count(
                len(region_nodes("boston"))
            )
            expect(page.locator("#mm-detail h2")).to_have_text(
                "Boston Board Game Night WG"
            )
            expect(page.locator("#mm-edges line")).to_have_count(edge_count("boston"))
            assert not console_errors, console_errors
            assert (
                page.locator("#mm-network").bounding_box()["x"]
                < page.locator("#mm-detail").bounding_box()["x"]
            )
            page.screenshot(path="/tmp/megamap-desktop-light.png", full_page=True)
            page.get_by_label("Search organizations or interests").fill("robot")
            expect(
                page.get_by_label("Search organizations or interests")
            ).to_be_focused()
            page.get_by_label("Category").select_option("robotics")
            matching = [
                n
                for n in region_nodes("boston")
                if n["category"] == "robotics"
                and "robot"
                in " ".join(
                    n[k] for k in ["name", "summary", "participation", "category"]
                ).lower()
            ]
            expect(page.locator("#mm-nodes button:visible")).to_have_count(
                len(matching)
            )
            page.get_by_label("Search organizations or interests").fill(
                "no-match-7c52834f"
            )
            expect(page.locator("#mm-nodes button:visible")).to_have_count(0)
            expect(page.locator("#mm-empty")).to_be_visible()
            expect(page.locator("#mm-detail")).to_contain_text(
                "No organization selected"
            )
            page.get_by_role("button", name="Reset filters").click()
            expect(page.locator("#mm-nodes button:visible")).to_have_count(
                len(region_nodes("boston"))
            )
            page.get_by_label("Category").select_option("social-play")
            page.locator('#mm-detail button[data-follow="massrobotics"]').click()
            expect(page.get_by_label("Category")).to_have_value("all")
            expect(page.locator("#mm-detail h2")).to_have_text("MassRobotics")
            page.locator('#mm-nodes button[data-node="robot-hackers"]').focus()
            page.keyboard.press("Enter")
            expect(page.locator("#mm-detail")).to_contain_text("Events shared by ←")
            expected_incoming = [
                e
                for e in data["edges"]
                if "robot-hackers" in (e["source"], e["target"])
            ]
            assert page.locator("#mm-detail .mm-relationship").count() == len(
                expected_incoming
            )
            assert (
                page.locator('#mm-detail .mm-relationship a[href^="https://"]').count()
                >= 2
            )
            page.get_by_label("Region").select_option("bay")
            expect(page.locator("#mm-nodes button:visible")).to_have_count(
                len(region_nodes("bay"))
            )
            expect(page.locator("#mm-detail")).to_contain_text("reviewed")
            expect(page.locator("#mm-detail")).not_to_contain_text("Research pending")
            expect(page.locator("#mm-edges line")).to_have_count(edge_count("bay"))
            page.get_by_role("button", name="Toggle Dark Mode").click()
            expect(page.locator("body")).to_have_attribute("data-theme", "dark")
            page.reload()
            expect(page.locator("body")).to_have_attribute("data-theme", "dark")
            expect(page.locator("#mm-app")).to_be_visible()
            page.set_viewport_size({"width": 390, "height": 844})
            expect(page.locator("#mm-nodes")).to_have_attribute(
                "aria-label", "Organizations"
            )
            expect(page.locator("#mm-nodes button:visible").first).to_be_visible()
            expect(page.locator("#mm-edges")).to_be_hidden()
            page.locator('#mm-nodes button[data-node="bu-rastic"]').click()
            expect(page.locator("#mm-detail h2")).to_be_in_viewport()
            expect(page.locator("#mm-detail h2")).to_be_focused()
            page.get_by_role("link", name="Back to organizations").click()
            expect(page.locator("#mm-network-title")).to_be_in_viewport()
            assert page.evaluate(
                "document.documentElement.scrollWidth <= innerWidth"
            ), "Mobile horizontal overflow"
            page.screenshot(path="/tmp/megamap-mobile.png", full_page=True)
            page.set_viewport_size({"width": 1440, "height": 1000})
            page.screenshot(path="/tmp/megamap-desktop.png", full_page=True)
            assert not errors, errors
            assert not console_errors, console_errors
            symmetric = page.locator("#mm-edges line.runs_activities_with").first
            expect(symmetric).to_have_attribute(
                "marker-start", "url(#mm-arrow-runs_activities_with)"
            )
            expect(symmetric).to_have_attribute(
                "marker-end", "url(#mm-arrow-runs_activities_with)"
            )
            expect(page.locator(".mm-legend li").nth(2)).to_have_text("Operates →")
            # Same canonical records are readable when enhancement never runs.
            nojs = browser.new_context(java_script_enabled=False)
            fallback = nojs.new_page()
            fallback.goto("http://127.0.0.1:8766/megamap/")
            expect(fallback.locator("#mm-directory")).to_be_visible()
            expect(fallback.locator("#mm-directory article")).to_have_count(
                len(data["nodes"])
            )
            expect(fallback.locator("#mm-directory .mm-relationship")).to_have_count(
                len(data["edges"])
            )
            expect(fallback.locator("#mm-app")).to_be_hidden()
            # Failed fetch leaves the directory visible, not a blank application.
            failure = context.new_page()
            failure.route("**/data/megamap.json", lambda route: route.abort())
            failure.goto("http://127.0.0.1:8766/megamap/")
            expect(failure.locator("#mm-load-status")).to_contain_text("directory")
            expect(failure.locator("#mm-directory")).to_be_visible()
            # Untrusted text stays literal; runtime links never allow executable URLs.
            hostile = json.loads(json.dumps(data))
            hostile["nodes"][0][
                "summary"
            ] = '<img src=x onerror="window.injected=true"></script>'
            hostile["nodes"][0]["sources"] = [
                {"label": "<b>unsafe</b>", "url": "javascript:window.injected=true"}
            ]
            security = context.new_page()
            security.route(
                "**/data/megamap.json", lambda route: route.fulfill(json=hostile)
            )
            security.goto("http://127.0.0.1:8766/megamap/")
            expect(security.locator("#mm-app")).to_be_visible()
            expect(security.locator("#mm-detail")).to_contain_text("<img src=x")
            assert (
                security.locator(
                    '#mm-detail img, #mm-detail script, #mm-detail a[href^="javascript:"]'
                ).count()
                == 0
            )
            assert security.evaluate("window.injected") is None
            # noindex is page-only, never applied to the homepage.
            page.goto("http://127.0.0.1:8766/")
            assert page.locator('meta[name="robots"][content*="noindex"]').count() == 0
            browser.close()
        print(
            "PASS: desktop, filters, relationships, keyboard, regions, theme, mobile, no-JS, fetch failure, XSS and unlisted route"
        )
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
