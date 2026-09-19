#!/usr/bin/env python3
"""Validate public Megadex data using Python's standard library only."""

import argparse
from datetime import date
import json
from pathlib import Path
import re
import sys
from urllib.parse import urlsplit

ID = re.compile(r"[a-z][a-z0-9]*(?:-[a-z0-9]+)*\Z")
COMPANY_FIELDS = {
    "id",
    "name",
    "region",
    "website",
    "careers_url",
    "summary",
    "news",
    "last_verified",
}
NEWS_FIELDS = {"date", "headline", "url"}


def safe_url(value):
    if not isinstance(value, str) or re.search(r'[\s\\\x00-\x1f\x7f<>"]', value):
        return False
    try:
        url = urlsplit(value)
        return (
            url.scheme in {"https", "http"}
            and bool(url.hostname)
            and url.username is None
            and url.password is None
            and url.port != 0
        )
    except ValueError:
        return False


def valid_date(value):
    if not isinstance(value, str) or not re.fullmatch(r"\d{4}-\d{2}-\d{2}", value):
        return False
    try:
        date.fromisoformat(value)
        return True
    except ValueError:
        return False


def validate(data):
    """Return actionable errors, never mutate the input; [] means valid."""
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)
        return condition

    def text(value, limit=2000):
        return isinstance(value, str) and bool(value.strip()) and len(value) <= limit

    if not check(isinstance(data, dict), "root: expected object"):
        return errors
    check(
        set(data) == {"version", "regions", "companies"},
        "root: unexpected or missing fields",
    )
    check(
        type(data.get("version")) is int and data["version"] == 1, "version: expected 1"
    )

    regions = data.get("regions")
    if check(
        isinstance(regions, dict) and regions, "regions: expected non-empty object"
    ):
        for key, label in regions.items():
            check(
                isinstance(key, str) and len(key) <= 40 and bool(ID.fullmatch(key)),
                f"regions: bad id {key!r}",
            )
            check(text(label, 120), f"regions[{key}]: bad label")

    companies = data.get("companies")
    if not check(isinstance(companies, list), "companies: expected list"):
        return errors

    seen_ids = set()
    today = date.today()
    for index, company in enumerate(companies):
        where = f"companies[{index}]"
        if not check(isinstance(company, dict), f"{where}: expected object"):
            continue
        where = f"companies[{index}]"
        extra = set(company) - COMPANY_FIELDS - {"location"}
        missing = COMPANY_FIELDS - set(company)
        check(not extra, f"{where}: unexpected fields {sorted(extra)}")
        check(not missing, f"{where}: missing fields {sorted(missing)}")

        cid = company.get("id")
        if check(
            isinstance(cid, str) and len(cid) <= 80 and bool(ID.fullmatch(cid or "")),
            f"{where}: bad id {cid!r}",
        ):
            check(cid not in seen_ids, f"{where}: duplicate id {cid!r}")
            seen_ids.add(cid)
            where = f"company {cid!r}"

        check(text(company.get("name"), 200), f"{where}: bad name")
        check(
            isinstance(regions, dict) and company.get("region") in regions,
            f"{where}: unknown region {company.get('region')!r}",
        )
        check(safe_url(company.get("website")), f"{where}: bad website URL")
        check(
            company.get("careers_url") is None or safe_url(company.get("careers_url")),
            f"{where}: bad careers_url",
        )
        check(
            text(company.get("summary"), 600), f"{where}: bad summary (max 600 chars)"
        )

        last = company.get("last_verified")
        check(
            valid_date(last) and date.fromisoformat(last) <= today,
            f"{where}: bad last_verified {last!r} (expected ISO date, not future)",
        )

        if "location" in company:
            location = company["location"]
            loc_where = f"{where} location"
            if check(isinstance(location, dict), f"{loc_where}: expected object"):
                check(
                    set(location)
                    == {"label", "lat", "lon", "precision", "source_url", "verified"},
                    f"{loc_where}: unexpected or missing fields",
                )
                check(text(location.get("label"), 200), f"{loc_where}: bad label")
                for field, limit in [("lat", 90), ("lon", 180)]:
                    value = location.get(field)
                    check(
                        type(value) in (int, float) and -limit <= value <= limit,
                        f"{loc_where}: bad {field} (finite number in [-{limit}, {limit}])",
                    )
                check(
                    location.get("precision") in ("city", "address"),
                    f"{loc_where}: bad precision",
                )
                source = location.get("source_url")
                check(
                    safe_url(source) and source.startswith("https://"),
                    f"{loc_where}: bad source_url (HTTPS required)",
                )
                verified = location.get("verified")
                check(
                    valid_date(verified) and date.fromisoformat(verified) <= today,
                    f"{loc_where}: bad verified (ISO date, not future)",
                )

        news = company.get("news")
        if check(isinstance(news, list), f"{where}: news expected list"):
            seen_news = set()
            for n_index, item in enumerate(news):
                n_where = f"{where} news[{n_index}]"
                if not check(isinstance(item, dict), f"{n_where}: expected object"):
                    continue
                check(
                    set(item) == NEWS_FIELDS,
                    f"{n_where}: expected fields {sorted(NEWS_FIELDS)}, got {sorted(item)}",
                )
                check(
                    valid_date(item.get("date")),
                    f"{n_where}: bad date {item.get('date')!r}",
                )
                check(text(item.get("headline"), 300), f"{n_where}: bad headline")
                url = item.get("url")
                check(safe_url(url), f"{n_where}: bad url")
                check(url not in seen_news, f"{n_where}: duplicate news url {url!r}")
                seen_news.add(url)
            dates = [str(item.get("date")) for item in news if isinstance(item, dict)]
            ok_dates = [d for d in dates if valid_date(d)]
            check(
                len(ok_dates) == len(dates)
                and ok_dates == sorted(ok_dates, reverse=True),
                f"{where}: news must be newest-first with valid ISO dates",
            )

    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        default=str(Path(__file__).resolve().parents[1] / "static/data/megadex.json"),
    )
    args = parser.parse_args()
    try:
        data = json.loads(Path(args.path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"megadex: cannot read {args.path}: {exc}", file=sys.stderr)
        return 1
    errors = validate(data)
    if errors:
        for error in errors:
            print(f"megadex error: {error}", file=sys.stderr)
        return 1
    companies = data.get("companies", [])
    print(f"megadex: valid ({len(companies)} companies)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
