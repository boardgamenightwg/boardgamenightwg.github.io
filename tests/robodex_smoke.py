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
from test_robodex import fixture, ValidatorTests, validate

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ROOT / "public"
ARTIFACTS = ROOT / "build/robodex-screenshots"
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
        data["companies"][0]["regions"] = ["boston", "bay"]
        data["companies"][0]["locations"] = {
            "boston": ValidatorTests.location(),
            "bay": ValidatorTests.location(
                label="Mountain View, CA", lat=37.386, lon=-122.0838
            ),
        }
        # Regional presence is known, but only Boston has a verified city.
        data["companies"][1]["regions"] = ["boston", "bay"]
        data["companies"][1]["locations"] = {"boston": ValidatorTests.location()}
        data["companies"].append(
            dict(
                data["companies"][0],
                id="unmapped",
                name="Unmapped fixture",
                regions=["unlocated"],
            )
        )
        del data["companies"][-1]["locations"]
        data["companies"].append(
            dict(
                data["companies"][0],
                id="bay-fixture",
                name="Bay fixture",
                regions=["bay"],
                careers_url=None,
                locations={
                    "bay": ValidatorTests.location(
                        label="Palo Alto, CA",
                        lat=37.4443,
                        lon=-122.1598,
                        precision="address",
                    )
                },
            )
        )
        # Valid hyphenated slugs must not alias: bay + area-acme vs bay-area + acme.
        data["regions"]["bay-area"] = "Hyphenated fixture region"
        data["companies"][0]["regions"].append("bay-area")
        data["companies"].insert(
            -1,
            dict(
                data["companies"][0],
                id="area-acme",
                name="Hyphenated ID fixture",
                regions=["bay"],
                locations={},
            ),
        )
        assert validate(data) == [], validate(data)
        (root / "static/data/robodex.json").write_text(json.dumps(data))
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
    page.goto(base + "/robodex/", wait_until="commit")
    expect(page.locator("#rdx-title")).to_be_visible()


def assert_source(page, data):
    expect(page.locator('meta[name="robots"]')).to_have_attribute(
        "content", "noindex,follow"
    )
    expect(page.locator("#rdx-title")).to_have_text("Robodex Experimental")
    assert page.title().startswith("Robodex | ")
    expect(page.locator('link[rel="canonical"]')).to_have_attribute(
        "href", page.url.split("#")[0].split("?")[0]
    )
    assert page.locator('header a[href*="robodex"]').count() == 0
    ids = page.locator("[id]").evaluate_all("nodes => nodes.map(n => n.id)")
    assert len(ids) == len(set(ids)), (
        "DOM IDs must be unique across regional views; duplicates: "
        f"{sorted({id_ for id_ in ids if ids.count(id_) > 1})}"
    )
    for region_id in data["regions"]:
        region = page.locator(f"#rdx-region-{region_id}")
        companies = [c for c in data["companies"] if region_id in c["regions"]]
        expect(region.locator(".rdx-entry")).to_have_count(len(companies))
        for number, company in enumerate(companies, 1):
            row = page.locator(f"#rdx-entry--{region_id}--{company['id']}")
            expect(row.locator(".rdx-number")).to_have_text(str(number))
            if company["careers_url"]:
                expect(row.locator("a.rdx-jobs")).to_have_attribute(
                    "href", company["careers_url"]
                )
            else:
                expect(row.locator("a.rdx-jobs")).to_have_count(0)
                expect(row.locator(".rdx-jobs-unavailable")).to_have_text(
                    "Jobs page not listed"
                )
            expect(row.locator("h3 a")).to_have_attribute("href", company["website"])
            expect(row.locator(".rdx-summary")).to_have_text(company["summary"])
            expect(row.locator(".rdx-verified time").first).to_have_attribute(
                "datetime", company["last_verified"]
            )
            expect(row.locator(".rdx-news li")).to_have_count(len(company["news"]))
            for item, news_row in zip(
                company["news"], row.locator(".rdx-news li").all()
            ):
                expect(news_row.locator("a")).to_have_text(item["headline"])
                expect(news_row.locator("a")).to_have_attribute("href", item["url"])
            dates = row.locator(".rdx-news time").evaluate_all(
                "nodes => nodes.map(n => n.dateTime)"
            )
            assert dates == [n["date"] for n in company["news"]]
            location = company.get("locations", {}).get(region_id)
            if location:
                expect(row.locator(".rdx-external-map")).to_have_attribute(
                    "href",
                    f"https://www.openstreetmap.org/?mlat={location['lat']}&mlon={location['lon']}#map=12/{location['lat']}/{location['lon']}",
                )
                expect(row.locator(".rdx-location")).to_contain_text(location["label"])
                if location["precision"] == "city":
                    expect(row.locator(".rdx-location")).to_contain_text(
                        "approximate city pin"
                    )
            else:
                assert row.get_attribute("data-lat") is None
                assert row.get_attribute("data-lon") is None
                expect(row.locator(".rdx-external-map")).to_have_count(0)
                expect(row.locator(".rdx-location")).to_have_text("Location not mapped")
                expect(row.locator(".rdx-map-button")).to_have_count(0)


def select_region(page, region_id):
    page.locator(f'[role="tab"][aria-controls="rdx-region-{region_id}"]').click()
    expect(page.locator(f"#rdx-region-{region_id}")).to_be_visible()


def assert_region_tabs(page, data):
    tabs = page.get_by_role("tab")
    expect(tabs).to_have_count(len(data["regions"]))
    expect(page.locator('[role="tabpanel"]:visible')).to_have_count(1)
    expect(page.locator("#rdx-region-boston")).to_be_visible()
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
        companies = [c for c in data["companies"] if region_id in c["regions"]]
        for number, company in enumerate(companies, 1):
            loc = company.get("locations", {}).get(region_id)
            if loc:
                groups.setdefault((loc["lat"], loc["lon"]), []).append(str(number))
        region = page.locator(f"#rdx-region-{region_id}")
        expect(region.locator(".rdx-marker")).to_have_count(len(groups))
        if groups:
            expect(region.locator(".rdx-map-status")).to_contain_text("Map ready")
            assert_markers_in_bounds(page)
            assert sorted(region.locator(".rdx-marker").all_text_contents()) == sorted(
                " · ".join(nums) for nums in groups.values()
            )


def assert_source_map_links(page, data):
    for region_id in data["regions"]:
        select_region(page, region_id)
        region = page.locator(f"#rdx-region-{region_id}")
        for company in data["companies"]:
            if region_id not in company["regions"] or region_id not in company.get(
                "locations", {}
            ):
                continue
            button = page.locator(
                f"#rdx-entry--{region_id}--{company['id']} .rdx-map-button"
            )
            button.click()
            selected = region.locator('.rdx-popup-company[aria-current="true"]')
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
        """() => [...document.querySelectorAll('.rdx-region:not([hidden]) .rdx-map:not([hidden])')].every(map => {
        const bounds = map.getBoundingClientRect();
        const markers = [...map.querySelectorAll('.rdx-marker')];
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
        page.locator(".rdx-region:visible").filter(has=page.locator(".rdx-entry")).first
    )
    listing, panel = region.locator(".rdx-list"), region.locator(".rdx-map-panel")
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
            row = listing.locator(".rdx-entry").first
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
    region = page.locator("#rdx-region-boston")
    # Analytics stays pending: no DOMContentLoaded dependency is permitted.
    marker = region.locator(".rdx-marker")
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
    assert_source(page, data)
    assert_maps(page, data)
    assert_source_map_links(page, data)
    # One canonical company has independent selection and focus in both views.
    select_region(page, "boston")
    page.locator("#rdx-entry--boston--other .rdx-map-button").click()
    page.keyboard.press("Escape")
    select_region(page, "bay")
    button = page.locator("#rdx-entry--bay--acme .rdx-map-button")
    button.focus()
    page.keyboard.press("Enter")
    expect(page.locator("#rdx-entry--bay--acme")).to_have_attribute(
        "aria-current", "true"
    )
    expect(page.locator("#rdx-entry--boston--other")).to_have_attribute(
        "aria-current", "true"
    )
    expect(page.locator("#rdx-entry--boston--acme")).not_to_have_attribute(
        "aria-current", "true"
    )
    expect(page.locator("#rdx-region-bay .leaflet-popup-content")).to_contain_text(
        "Mountain View, CA"
    )
    page.keyboard.press("Escape")
    expect(button).to_be_focused()
    select_region(page, "boston")
    expect(page.locator("#rdx-entry--boston--other")).to_have_attribute(
        "aria-current", "true"
    )
    select_region(page, "empty")
    expect(page.locator("#rdx-region-empty .rdx-empty")).to_be_visible()
    select_region(page, "unlocated")
    expect(page.locator("#rdx-region-unlocated .rdx-map-status")).to_contain_text(
        "No verified locations"
    )
    expect(page.locator("#rdx-region-unlocated .rdx-map")).to_be_hidden()
    select_region(page, "bay")
    expect(page.locator("#rdx-region-bay .rdx-marker")).to_have_count(2)
    page.locator("#rdx-entry--bay--bay-fixture .rdx-map-button").click()
    popup = page.locator("#rdx-region-bay .leaflet-popup-content")
    expect(popup.locator(".rdx-jobs")).to_have_count(0)
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
    expect(page.locator("#rdx-entry--bay--bay-fixture .rdx-map-button")).to_be_focused()
    select_region(page, "boston")
    assert_source(page, data)
    assert_layout(page)
    status = region.locator(".rdx-map-status")
    expect(status).to_contain_text("Map ready")
    assert (
        status.evaluate("el => getComputedStyle(el).position") == "absolute"
    ), "Ready status should not duplicate visible help"
    marker.focus()
    page.keyboard.press("Enter")
    expect(page.locator("#rdx-entry--boston--acme")).to_have_attribute(
        "aria-current", "true"
    )
    expect(region.locator(".leaflet-popup-content img")).to_have_count(0)
    expect(region.locator(".leaflet-popup-content a.rdx-jobs")).to_have_count(2)
    page.locator("#rdx-entry--boston--other .rdx-map-button").click()
    expect(page.locator("#rdx-entry--boston--other")).to_have_attribute(
        "aria-current", "true"
    )
    expect(page.locator("#rdx-entry--boston--acme")).not_to_have_attribute(
        "aria-current", "true"
    )
    expect(region.locator('.rdx-popup-company[aria-current="true"]')).to_contain_text(
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
    expect(page.locator("#rdx-entry--boston--acme")).to_have_attribute(
        "aria-current", "true"
    )
    page.locator("#rdx-entry--boston--other .rdx-map-button").click()
    marker.focus()
    page.keyboard.press("Space")
    expect(page.locator("#rdx-entry--boston--acme")).to_have_attribute(
        "aria-current", "true"
    )
    page.set_viewport_size({"width": 390, "height": 844})
    page.locator("#rdx-entry--boston--other .rdx-map-button").focus()
    page.keyboard.press("Enter")
    expect(
        region.locator('.rdx-popup-company[aria-current="true"] .rdx-jobs')
    ).to_be_focused()
    page.keyboard.press("Escape")
    expect(page.locator("#rdx-entry--boston--other .rdx-map-button")).to_be_focused()
    page.keyboard.press("Enter")
    # A container resize must preserve the open popup, not refit it behind the clip.
    page.set_viewport_size({"width": 392, "height": 844})
    page.wait_for_timeout(300)  # Let Leaflet's popup auto-pan finish.
    geometry = region.locator(".rdx-map").evaluate(
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
        region.locator('.rdx-popup-company[aria-current="true"] .rdx-jobs')
    ).to_be_focused()
    region.locator(".leaflet-popup-close-button").click()
    expect(page.locator("#rdx-entry--boston--other .rdx-map-button")).to_be_focused()
    page.keyboard.press("Enter")
    marker.click()
    page.wait_for_function(
        """() => {
        const popup = document.querySelector('#rdx-region-boston .leaflet-popup').getBoundingClientRect();
        return popup.top >= 0 && popup.bottom <= innerHeight;
    }""",
        timeout=3000,
    )
    expect(page.locator("#rdx-entry--boston--acme")).to_have_attribute(
        "aria-current", "true"
    )
    page.set_viewport_size({"width": 1440, "height": 1000})
    # Genuine map interaction, not a static graphic: zoom buttons alter the tiles.
    zoom = region.locator(".leaflet-control-zoom-in")
    before = region.locator(".leaflet-tile").first.get_attribute("src")
    zoom.click()
    page.wait_for_function(
        "old => !Array.from(document.querySelectorAll('#rdx-region-boston .leaflet-tile')).some(n => n.src === old)",
        arg=before,
    )
    tiles_before = region.locator(".leaflet-tile").evaluate_all(
        "nodes => nodes.map(n => n.src)"
    )
    region.locator(".rdx-map").hover()
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
            expect(page.locator(".rdx-region:visible")).to_have_count(
                len(data["regions"])
            )
        else:
            assert_region_tabs(page, data)
        status = page.locator("#rdx-region-boston .rdx-map-status")
        expect(status).to_contain_text("Map unavailable")
        if failure == "tiles":
            expect(status).to_contain_text("tiles")
            page.locator("#rdx-entry--boston--other .rdx-map-button").click()
            expect(page.locator("#rdx-entry--boston--other")).to_have_attribute(
                "aria-current", "true"
            )
        else:
            expect(
                page.locator("#rdx-entry--boston--other .rdx-map-button")
            ).to_be_hidden()
        page.screenshot(
            path=str(ARTIFACTS / f"robodex-fixture-{failure}.png"), full_page=True
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
    mapped = [(c, r) for c in data["companies"] for r in c.get("locations", {})]
    assert mapped, "Live QA needs at least one verified source location"
    for region_id in {r for _, r in mapped}:
        select_region(page, region_id)
        region = page.locator(f"#rdx-region-{region_id}")
        expect(region.locator(".rdx-map-status")).to_contain_text(
            "Map ready", timeout=30000
        )
        assert region.locator(".leaflet-tile-loaded").count() > 0
        expect(
            region.locator(
                '.leaflet-control-attribution a[href="https://www.openstreetmap.org/copyright"]'
            )
        ).to_be_visible()
    assert tile_responses and all(s == 200 for s in tile_responses), tile_responses
    page.screenshot(path=str(ARTIFACTS / "robodex-live-desktop.png"), full_page=True)
    company, region_id = mapped[0]
    select_region(page, region_id)
    page.locator(f"#rdx-entry--{region_id}--{company['id']} .rdx-map-button").click()
    expect(page.locator(f"#rdx-entry--{region_id}--{company['id']}")).to_have_attribute(
        "aria-current", "true"
    )
    page.screenshot(path=str(ARTIFACTS / "robodex-live-selected.png"), full_page=True)
    page.set_viewport_size({"width": 390, "height": 844})
    page.screenshot(path=str(ARTIFACTS / "robodex-live-mobile.png"), full_page=True)
    page.set_viewport_size({"width": 1440, "height": 1000})
    page.locator(".theme-toggle").click()
    expect(page.locator("body")).to_have_attribute("data-theme", "dark")
    page.screenshot(path=str(ARTIFACTS / "robodex-live-dark.png"), full_page=True)
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
    data = json.loads((ROOT / "static/data/robodex.json").read_text())
    sitemap = (PUBLIC / "sitemap.xml").read_text()
    assert "robodex" not in sitemap
    with sync_playwright() as p:
        browser = p.chromium.launch()
        with serve(PUBLIC, 8767) as base:
            context, page, errors = setup(browser)
            open_page(page, base)
            assert_source(page, data)
            assert_region_tabs(page, data)
            assert_maps(page, data)
            # Overview bounds apply before popup auto-pan changes the user's view.
            # Popup/link interaction checks intentionally pan to each selection.
            assert_layout(page)
            assert_source_map_links(page, data)
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
