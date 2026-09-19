"""Built-page checks + deterministic map regressions; --live-tiles adds network QA.

Default tests use real vendored Leaflet with mocked tile images, not live map QA.
Fixtures build in build/ without ever changing the canonical source data.
"""

import argparse
from contextlib import contextmanager
import functools
import http.server
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
from playwright.sync_api import sync_playwright, expect
from test_megadex import fixture, ValidatorTests

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ROOT / "public"
ARTIFACTS = ROOT / "build/megadex-screenshots"
TILES = "https://tile.openstreetmap.org/**"
# Explicit synthetic tile only for offline interaction tests; never screenshot it.
TILE = '<svg xmlns="http://www.w3.org/2000/svg" width="256" height="256"><rect width="256" height="256" fill="#eee"/></svg>'


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


@contextmanager
def serve(directory, port=0):
    handler = functools.partial(QuietHandler, directory=str(directory))
    server = http.server.ThreadingHTTPServer(("127.0.0.1", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        yield f"http://127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        server.server_close()


@contextmanager
def fixture_build():
    (ROOT / "build").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=ROOT / "build") as directory:
        root = Path(directory)
        shutil.copy(ROOT / "config.toml", root)
        for name in ["templates", "content", "static"]:
            shutil.copytree(ROOT / name, root / name)
        data = fixture()
        data["regions"]["empty"] = "Empty fixture region"
        data["regions"]["unlocated"] = "Unmapped fixture region"
        data["companies"][0]["name"] = 'Fixture <img src=x onerror="alert(1)"> Robotics'
        data["companies"][0]["location"] = ValidatorTests.location()
        data["companies"][1]["location"] = ValidatorTests.location()
        data["companies"].append(
            dict(
                data["companies"][0],
                id="unmapped",
                name="Unmapped fixture",
                region="unlocated",
            )
        )
        del data["companies"][-1]["location"]
        data["companies"].append(
            dict(
                data["companies"][0],
                id="bay-fixture",
                name="Bay fixture",
                region="bay",
                careers_url=None,
                location=ValidatorTests.location(
                    label="Mountain View, CA",
                    lat=37.386,
                    lon=-122.0838,
                    precision="address",
                ),
            )
        )
        (root / "static/data/megadex.json").write_text(json.dumps(data))
        # Relative base URL resolved at serving time; production build still uses 8767.
        subprocess.run(
            [
                "zola",
                "--root",
                str(root),
                "build",
                "--base-url",
                "http://127.0.0.1:8767",
            ],
            check=True,
        )
        yield root / "public", data


def setup(browser, javascript=True, tiles="mock", pending_analytics=False):
    context = browser.new_context(
        viewport={"width": 1440, "height": 1000}, java_script_enabled=javascript
    )
    pending = []
    context.route(
        "https://pls.mrkaran.dev/**",
        (lambda route: pending.append(route))
        if pending_analytics
        else (lambda route: route.abort()),
    )
    if tiles == "mock":
        context.route(
            TILES,
            lambda route: route.fulfill(
                status=200, content_type="image/svg+xml", body=TILE
            ),
        )
    elif tiles == "blocked":
        context.route(TILES, lambda route: route.abort())
    context._pending_analytics = pending
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    return context, page, errors


def open_page(page, base):
    page.goto(base + "/megadex/", wait_until="commit")
    expect(page.locator("#mdx-title")).to_be_visible()


def assert_source(page, data):
    expect(page.locator('meta[name="robots"]')).to_have_attribute(
        "content", "noindex,follow"
    )
    assert page.locator('header a[href*="megadex"]').count() == 0
    for region_id in data["regions"]:
        region = page.locator(f"#mdx-region-{region_id}")
        companies = [c for c in data["companies"] if c["region"] == region_id]
        expect(region.locator(".mdx-entry")).to_have_count(len(companies))
        for number, company in enumerate(companies, 1):
            row = page.locator(f"#mdx-{company['id']}")
            expect(row.locator(".mdx-number")).to_have_text(str(number))
            if company["careers_url"]:
                expect(row.locator("a.mdx-jobs")).to_have_attribute(
                    "href", company["careers_url"]
                )
            else:
                expect(row.locator("a.mdx-jobs")).to_have_count(0)
                expect(row.locator(".mdx-jobs-unavailable")).to_have_text(
                    "Jobs page not listed"
                )
            expect(row.locator("h3 a")).to_have_attribute("href", company["website"])
            expect(row.locator(".mdx-news li")).to_have_count(len(company["news"]))
            dates = row.locator(".mdx-news time").evaluate_all(
                "nodes => nodes.map(n => n.dateTime)"
            )
            assert dates == [n["date"] for n in company["news"]]
            if "location" in company:
                expect(row.locator(".mdx-external-map")).to_have_attribute(
                    "href",
                    f"https://www.openstreetmap.org/?mlat={company['location']['lat']}&mlon={company['location']['lon']}#map=12/{company['location']['lat']}/{company['location']['lon']}",
                )
                expect(row.locator(".mdx-location")).to_contain_text(
                    company["location"]["label"]
                )
                if company["location"]["precision"] == "city":
                    expect(row.locator(".mdx-location")).to_contain_text(
                        "approximate city pin"
                    )
            else:
                expect(row.locator(".mdx-location")).to_have_text("Location not mapped")
                expect(row.locator(".mdx-map-button")).to_have_count(0)


def select_region(page, region_id):
    page.locator(f'[role="tab"][aria-controls="mdx-region-{region_id}"]').click()
    expect(page.locator(f"#mdx-region-{region_id}")).to_be_visible()


def assert_region_tabs(page, data):
    tabs = page.get_by_role("tab")
    expect(tabs).to_have_count(len(data["regions"]))
    expect(page.locator('[role="tabpanel"]:visible')).to_have_count(1)
    expect(page.locator("#mdx-region-boston")).to_be_visible()
    for region_id, label in data["regions"].items():
        select_region(page, region_id)
        expect(page.get_by_role("tab", name=label, exact=True)).to_have_attribute(
            "aria-selected", "true"
        )
        expect(page.locator('[role="tabpanel"]:visible')).to_have_count(1)
    tabs.first.focus()
    page.keyboard.press("End")
    expect(tabs.last).to_be_focused()
    expect(tabs.last).to_have_attribute("aria-selected", "true")
    page.keyboard.press("ArrowRight")
    expect(tabs.first).to_be_focused()
    expect(tabs.first).to_have_attribute("aria-selected", "true")
    page.keyboard.press("ArrowLeft")
    expect(tabs.last).to_be_focused()
    page.keyboard.press("Home")
    expect(tabs.first).to_be_focused()
    select_region(page, "boston")


def assert_maps(page, data):
    for region_id in data["regions"]:
        select_region(page, region_id)
        groups = {}
        companies = [c for c in data["companies"] if c["region"] == region_id]
        for number, company in enumerate(companies, 1):
            if company.get("location"):
                loc = company["location"]
                groups.setdefault((loc["lat"], loc["lon"]), []).append(str(number))
        region = page.locator(f"#mdx-region-{region_id}")
        expect(region.locator(".mdx-marker")).to_have_count(len(groups))
        if groups:
            expect(region.locator(".mdx-map-status")).to_contain_text("Map ready")
            assert_markers_in_bounds(page)
            assert sorted(region.locator(".mdx-marker").all_text_contents()) == sorted(
                " · ".join(nums) for nums in groups.values()
            )


def assert_source_map_links(page, data):
    for region_id in data["regions"]:
        select_region(page, region_id)
        region = page.locator(f"#mdx-region-{region_id}")
        for company in data["companies"]:
            if company["region"] != region_id or not company.get("location"):
                continue
            button = page.locator(f"#mdx-{company['id']} .mdx-map-button")
            button.click()
            selected = region.locator('.mdx-popup-company[aria-current="true"]')
            expect(selected.locator("strong")).to_contain_text(company["name"])
            expect(selected.locator("a")).to_have_attribute(
                "href", company["careers_url"] or company["website"]
            )
            expect(selected.locator("a")).to_be_focused()
            expect(selected.locator("a")).to_be_in_viewport()
            region.locator(".leaflet-popup-close-button").click(timeout=4000)
            expect(region.locator(".leaflet-popup")).to_have_count(0)
            expect(button).to_be_focused()


def assert_markers_in_bounds(page):
    page.wait_for_function(
        """() => [...document.querySelectorAll('.mdx-region:not([hidden]) .mdx-map:not([hidden])')].every(map => {
        const bounds = map.getBoundingClientRect();
        const markers = [...map.querySelectorAll('.mdx-marker')];
        return markers.length > 0 && markers.every(marker => {
            const pin = marker.getBoundingClientRect();
            return pin.left >= bounds.left && pin.right <= bounds.right &&
                   pin.top >= bounds.top && pin.bottom <= bounds.bottom;
        });
    })""",
        timeout=5000,
    )


def assert_layout(page):
    region = (
        page.locator(".mdx-region:visible").filter(has=page.locator(".mdx-entry")).first
    )
    listing, panel = region.locator(".mdx-list"), region.locator(".mdx-map-panel")
    for width in [1440, 390, 320, 1440]:
        page.set_viewport_size({"width": width, "height": 1000})
        assert_markers_in_bounds(page)
        assert page.evaluate(
            "document.documentElement.scrollWidth <= innerWidth"
        ), f"Overflow at {width}px"
        left, right = listing.bounding_box(), panel.bounding_box()
        if width == 1440:
            assert (
                left["x"] + left["width"] <= right["x"]
            ), "Desktop map must sit beside list"
            row = listing.locator(".mdx-entry").first
            assert (
                row.evaluate("el => parseFloat(getComputedStyle(el).paddingTop)") <= 12
            )
            assert listing.evaluate("el => getComputedStyle(el).rowGap") in [
                "normal",
                "0px",
            ]
        else:
            assert (
                right["y"] + right["height"] <= left["y"]
            ), "Mobile map must stack above list"
    page.set_viewport_size({"width": 1440, "height": 1000})


def fixture_checks(browser, base, data):
    context, page, errors = setup(browser, pending_analytics=True)
    open_page(page, base)
    region = page.locator("#mdx-region-boston")
    # Analytics stays pending: no DOMContentLoaded dependency is permitted.
    marker = region.locator(".mdx-marker")
    expect(marker).to_have_count(1)
    assert context._pending_analytics, "Analytics request must actually be pending"
    assert page.evaluate("document.readyState") == "interactive"
    for route in context._pending_analytics:
        route.abort()
    expect(marker).to_have_attribute("role", "button")
    expect(marker).to_have_attribute("tabindex", "0")
    expect(marker).to_have_text("1 · 2")
    expect(marker).to_have_attribute(
        "aria-label", '1: Fixture <img src=x onerror="alert(1)"> Robotics; 2: Other Co'
    )
    assert_region_tabs(page, data)
    select_region(page, "empty")
    expect(page.locator("#mdx-region-empty .mdx-empty")).to_be_visible()
    select_region(page, "unlocated")
    expect(page.locator("#mdx-region-unlocated .mdx-map-status")).to_contain_text(
        "No verified locations"
    )
    expect(page.locator("#mdx-region-unlocated .mdx-map")).to_be_hidden()
    select_region(page, "bay")
    expect(page.locator("#mdx-region-bay .mdx-marker")).to_have_count(1)
    page.locator("#mdx-bay-fixture .mdx-map-button").click()
    popup = page.locator("#mdx-region-bay .leaflet-popup-content")
    expect(popup.locator(".mdx-jobs")).to_have_count(0)
    expect(popup.get_by_role("link", name="Website →")).to_have_attribute(
        "href", data["companies"][-1]["website"]
    )
    expect(popup.get_by_role("link", name="Website →")).to_be_focused()
    page.evaluate("document.body.dataset.theme = 'dark'")
    assert (
        popup.get_by_role("link", name="Website →").evaluate(
            "el => getComputedStyle(el).color"
        )
        == "rgb(36, 88, 166)"
    )
    page.evaluate("document.body.dataset.theme = 'light'")
    page.keyboard.press("Escape")
    expect(page.locator("#mdx-bay-fixture .mdx-map-button")).to_be_focused()
    select_region(page, "boston")
    assert_source(page, data)
    assert_layout(page)
    status = region.locator(".mdx-map-status")
    expect(status).to_contain_text("Map ready")
    assert (
        status.evaluate("el => getComputedStyle(el).position") == "absolute"
    ), "Ready status should not duplicate visible help"
    marker.focus()
    page.keyboard.press("Enter")
    expect(page.locator("#mdx-acme")).to_have_attribute("aria-current", "true")
    expect(region.locator(".leaflet-popup-content img")).to_have_count(0)
    expect(region.locator(".leaflet-popup-content a.mdx-jobs")).to_have_count(2)
    page.locator("#mdx-other .mdx-map-button").click()
    expect(page.locator("#mdx-other")).to_have_attribute("aria-current", "true")
    expect(page.locator("#mdx-acme")).not_to_have_attribute("aria-current", "true")
    expect(region.locator('.mdx-popup-company[aria-current="true"]')).to_contain_text(
        "Other Co"
    )
    expect(region.locator(".leaflet-popup-content")).to_contain_text(
        "approximate city pin"
    )
    page.locator(".theme-toggle").click()
    expect(page.locator("body")).to_have_attribute("data-theme", "dark")
    assert (
        region.locator(".leaflet-popup-content strong").first.evaluate(
            "el => getComputedStyle(el).color"
        )
        == "rgb(34, 34, 34)"
    ), "Dark theme must retain legible dark popup headings on white"
    assert (
        region.locator(".leaflet-control-zoom-in").evaluate(
            "el => getComputedStyle(el).color"
        )
        == "rgb(34, 34, 34)"
    )
    marker.click()
    expect(page.locator("#mdx-acme")).to_have_attribute("aria-current", "true")
    page.locator("#mdx-other .mdx-map-button").click()
    marker.focus()
    page.keyboard.press("Space")
    expect(page.locator("#mdx-acme")).to_have_attribute("aria-current", "true")
    page.set_viewport_size({"width": 390, "height": 844})
    page.locator("#mdx-other .mdx-map-button").focus()
    page.keyboard.press("Enter")
    expect(
        region.locator('.mdx-popup-company[aria-current="true"] .mdx-jobs')
    ).to_be_focused()
    page.keyboard.press("Escape")
    expect(page.locator("#mdx-other .mdx-map-button")).to_be_focused()
    page.keyboard.press("Enter")
    # A container resize must preserve the open popup, not refit it behind the clip.
    page.set_viewport_size({"width": 392, "height": 844})
    page.wait_for_timeout(300)  # Let Leaflet's popup auto-pan finish.
    geometry = region.locator(".mdx-map").evaluate(
        """map => {
        const close = map.querySelector('.leaflet-popup-close-button');
        const m = map.getBoundingClientRect(), c = close.getBoundingClientRect();
        return {scrollTop: map.scrollTop, scrollLeft: map.scrollLeft,
            map: m.toJSON(), close: c.toJSON(),
            clickable: close.contains(document.elementFromPoint(c.x + c.width / 2, c.y + c.height / 2))};
        }"""
    )
    assert geometry["scrollTop"] == 0 and geometry["scrollLeft"] == 0, geometry
    assert geometry["clickable"], geometry
    expect(
        region.locator('.mdx-popup-company[aria-current="true"] .mdx-jobs')
    ).to_be_focused()
    region.locator(".leaflet-popup-close-button").click()
    expect(page.locator("#mdx-other .mdx-map-button")).to_be_focused()
    page.keyboard.press("Enter")
    marker.click()
    page.wait_for_function(
        """() => {
        const popup = document.querySelector('#mdx-region-boston .leaflet-popup').getBoundingClientRect();
        return popup.top >= 0 && popup.bottom <= innerHeight;
    }""",
        timeout=3000,
    )
    expect(page.locator("#mdx-acme")).to_have_attribute("aria-current", "true")
    page.set_viewport_size({"width": 1440, "height": 1000})
    # Genuine map interaction, not a static graphic: zoom buttons alter the tiles.
    zoom = region.locator(".leaflet-control-zoom-in")
    before = region.locator(".leaflet-tile").first.get_attribute("src")
    zoom.click()
    page.wait_for_function(
        "old => !Array.from(document.querySelectorAll('#mdx-region-boston .leaflet-tile')).some(n => n.src === old)",
        arg=before,
    )
    tiles_before = region.locator(".leaflet-tile").evaluate_all(
        "nodes => nodes.map(n => n.src)"
    )
    region.locator(".mdx-map").hover()
    page.mouse.wheel(0, 400)
    page.wait_for_timeout(350)
    assert tiles_before == region.locator(".leaflet-tile").evaluate_all(
        "nodes => nodes.map(n => n.src)"
    ), "Wheel must not zoom map"
    assert not errors, errors
    context.close()

    for failure in ["no-js", "library", "tiles"]:
        context, page, errors = setup(
            browser, javascript=failure != "no-js", tiles="blocked"
        )
        if failure == "library":
            context.route("**/vendor/leaflet/leaflet.js", lambda route: route.abort())
        open_page(page, base)
        assert_source(page, data)
        if failure == "no-js":
            expect(page.get_by_role("tab")).to_have_count(0)
            expect(page.locator(".mdx-region:visible")).to_have_count(
                len(data["regions"])
            )
        else:
            assert_region_tabs(page, data)
        status = page.locator("#mdx-region-boston .mdx-map-status")
        expect(status).to_contain_text("Map unavailable")
        if failure == "tiles":
            expect(status).to_contain_text("tiles")
            page.locator("#mdx-other .mdx-map-button").click()
            expect(page.locator("#mdx-other")).to_have_attribute("aria-current", "true")
        else:
            expect(page.locator("#mdx-other .mdx-map-button")).to_be_hidden()
        page.screenshot(
            path=str(ARTIFACTS / f"megadex-fixture-{failure}.png"), full_page=True
        )
        assert not errors, errors
        context.close()
    print(
        "PASS: deterministic fixture / real Leaflet + MOCK tiles; co-located pins, keyboard, popup safety, selection, zoom, wheel, pending analytics, empty/missing locations, failures"
    )


def live_checks(browser, base, data):
    context, page, errors = setup(browser, tiles="live")
    tile_responses = []
    page.on(
        "response",
        lambda response: tile_responses.append(response.status)
        if response.url.startswith("https://tile.openstreetmap.org/")
        else None,
    )
    open_page(page, base)
    mapped = [c for c in data["companies"] if c.get("location")]
    assert mapped, "Live QA needs at least one verified source location"
    for region_id in {c["region"] for c in mapped}:
        select_region(page, region_id)
        region = page.locator(f"#mdx-region-{region_id}")
        expect(region.locator(".mdx-map-status")).to_contain_text(
            "Map ready", timeout=30000
        )
        assert region.locator(".leaflet-tile-loaded").count() > 0
        expect(
            region.locator(
                '.leaflet-control-attribution a[href="https://www.openstreetmap.org/copyright"]'
            )
        ).to_be_visible()
    assert tile_responses and all(s == 200 for s in tile_responses), tile_responses
    page.screenshot(path=str(ARTIFACTS / "megadex-live-desktop.png"), full_page=True)
    company = mapped[0]
    select_region(page, company["region"])
    page.locator(f"#mdx-{company['id']} .mdx-map-button").click()
    expect(page.locator(f"#mdx-{company['id']}")).to_have_attribute(
        "aria-current", "true"
    )
    page.screenshot(path=str(ARTIFACTS / "megadex-live-selected.png"), full_page=True)
    page.set_viewport_size({"width": 390, "height": 844})
    page.screenshot(path=str(ARTIFACTS / "megadex-live-mobile.png"), full_page=True)
    page.set_viewport_size({"width": 1440, "height": 1000})
    page.locator(".theme-toggle").click()
    expect(page.locator("body")).to_have_attribute("data-theme", "dark")
    page.screenshot(path=str(ARTIFACTS / "megadex-live-dark.png"), full_page=True)
    assert not errors, errors
    context.close()
    print(
        f"PASS: LIVE OpenStreetMap tiles ({len(tile_responses)} HTTP 200 responses), source-data locations, screenshots"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-tiles", action="store_true")
    args = parser.parse_args()
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    data = json.loads((ROOT / "static/data/megadex.json").read_text())
    assert "megadex" not in (PUBLIC / "sitemap.xml").read_text()
    with sync_playwright() as p:
        browser = p.chromium.launch()
        with serve(PUBLIC, 8767) as base:
            context, page, errors = setup(browser)
            open_page(page, base)
            assert_source(page, data)
            assert_region_tabs(page, data)
            assert_maps(page, data)
            assert_source_map_links(page, data)
            assert_layout(page)
            mobile = context.new_page()
            mobile.set_viewport_size({"width": 390, "height": 844})
            open_page(mobile, base)
            assert_maps(mobile, data)
            assert_markers_in_bounds(mobile)
            mobile.close()
            page.goto(base + "/")
            assert page.locator('meta[name="robots"][content*="noindex"]').count() == 0
            assert not errors, errors
            context.close()
            print(
                "PASS: source-driven directory, news, links, noindex/unlisted, compact responsive layout"
            )
            if args.live_tiles:
                live_checks(browser, base, data)
        with fixture_build() as (public, fixture_data):
            with serve(public, 8767) as base:
                fixture_checks(browser, base, fixture_data)
        browser.close()


if __name__ == "__main__":
    main()
