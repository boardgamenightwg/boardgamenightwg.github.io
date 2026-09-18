"""Optional browser smoke for the server-rendered Megadex; build to public/ first."""

import functools
import http.server
import json
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ROOT / "public"


def main():
    assert (PUBLIC / "megadex/index.html").is_file(), "Missing built Megadex route"
    data = json.loads((ROOT / "static/data/megadex.json").read_text())
    region_companies = lambda region: [
        c for c in data["companies"] if c["region"] == region
    ]

    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=str(PUBLIC)
    )
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 8767), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch()
            context = browser.new_context(viewport={"width": 1440, "height": 1000})
            page = context.new_page()
            errors = []
            console_errors = []
            context.route("https://pls.mrkaran.dev/**", lambda route: route.abort())
            page.on(
                "console",
                lambda msg: (
                    console_errors.append(msg.text)
                    if msg.type == "error"
                    and not msg.location.get("url", "").startswith(
                        "https://pls.mrkaran.dev/"
                    )
                    else None
                ),
            )
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.goto("http://127.0.0.1:8767/megadex/", wait_until="domcontentloaded")

            # Unlisted route: noindex page-only, no sitemap entry, no nav link.
            expect(page.locator('meta[name="robots"]')).to_have_attribute(
                "content", "noindex,follow"
            )
            sitemap = (PUBLIC / "sitemap.xml").read_text()
            assert "megadex" not in sitemap
            assert page.locator('header a[href*="megadex"]').count() == 0

            # Every region section renders its companies from the data file.
            for region_id in data["regions"]:
                expect(page.locator(f"#mdx-region-{region_id}")).to_be_visible()
                expect(
                    page.locator(f"#mdx-region-{region_id} .mdx-entry")
                ).to_have_count(len(region_companies(region_id)))

            # Every company entry has a careers link and website link.
            for company in data["companies"]:
                entry = page.locator(f"#mdx-{company['id']}")
                expect(entry).to_be_visible()
                expect(entry.locator("a.mdx-jobs")).to_have_attribute(
                    "href", company["careers_url"]
                )
                expect(entry.locator("h3 a")).to_have_attribute(
                    "href", company["website"]
                )

            # News renders newest-first when present.
            with_news = [c for c in data["companies"] if c["news"]]
            for company in with_news:
                items = page.locator(f"#mdx-{company['id']} .mdx-news li")
                expect(items).to_have_count(len(company["news"]))
                rendered = [
                    items.nth(i).locator("time").get_attribute("datetime")
                    for i in range(len(company["news"]))
                ]
                assert rendered == [n["date"] for n in company["news"]]

            page.screenshot(path="/tmp/megadex-desktop.png", full_page=True)

            # Mobile: no horizontal overflow.
            page.set_viewport_size({"width": 390, "height": 844})
            assert page.evaluate(
                "document.documentElement.scrollWidth <= innerWidth"
            ), "Mobile horizontal overflow"
            page.screenshot(path="/tmp/megadex-mobile.png", full_page=True)

            # noindex is page-only, never applied to the homepage.
            page.goto("http://127.0.0.1:8767/")
            assert page.locator('meta[name="robots"][content*="noindex"]').count() == 0

            assert not errors, errors
            assert not console_errors, console_errors
            browser.close()
        print(
            "PASS: rendered regions, careers links, news order, unlisted route, mobile"
        )
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
