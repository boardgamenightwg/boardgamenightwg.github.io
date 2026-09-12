#!/usr/bin/env python3
"""Validate public Megamap data using Python's standard library only."""
import argparse
from datetime import date
import json
from pathlib import Path
import re
import sys
from urllib.parse import urlsplit

ID = re.compile(r"[a-z][a-z0-9]*(?:-[a-z0-9]+)*\Z")
TYPES = {"shares_events_from", "runs_activities_with", "operates"}
COMMON = {"id", "sources", "last_verified", "status"}
NODE_FIELDS = COMMON | {"name", "region", "category", "summary", "participation"}
EDGE_FIELDS = COMMON | {"source", "target", "type", "description"}


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


def validate(data):
    """Return actionable errors, never mutate the input; [] means valid."""
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)
        return condition

    def text(value, limit=2000):
        return isinstance(value, str) and bool(value.strip()) and len(value) <= limit

    def identifier(value):
        return isinstance(value, str) and len(value) <= 80 and bool(ID.fullmatch(value))

    if not check(isinstance(data, dict), "root: expected object"):
        return errors
    check(
        set(data) == {"version", "regions", "categories", "nodes", "edges"},
        "root: unexpected or missing fields",
    )
    check(
        type(data.get("version")) is int and data["version"] == 1, "version: expected 1"
    )
    catalogs = {}
    for key in ["regions", "categories"]:
        value = data.get(key)
        if check(
            isinstance(value, dict) and bool(value),
            f"{key}: expected nonempty ID-to-label object",
        ):
            for k, v in value.items():
                check(identifier(k) and text(v, 80), f"{key}.{k}: invalid ID or label")
            if key == "categories":
                check(
                    "all" not in value,
                    "categories.all: reserved for the all-categories filter",
                )
            catalogs[key] = value
        else:
            catalogs[key] = {}

    records = {}
    for kind, fields in [("nodes", NODE_FIELDS), ("edges", EDGE_FIELDS)]:
        items = data.get(kind)
        if not check(isinstance(items, list), f"{kind}: expected array"):
            items = []
        if kind == "nodes":
            check(bool(items), "nodes: at least one organization required")
        records[kind] = []
        ids = set()
        for index, item in enumerate(items):
            where = f"{kind}[{index}]"
            if not check(isinstance(item, dict), f"{where}: expected object"):
                continue
            records[kind].append(item)
            check(
                set(item) == fields,
                f"{where}: missing/unknown fields (expected {sorted(fields)})",
            )
            ident = item.get("id")
            if check(
                identifier(ident), f"{where}.id: use a stable lowercase hyphenated ID"
            ):
                check(ident not in ids, f"{where}.id: duplicate {ident}")
                ids.add(ident)
            statuses = (
                {"reviewed", "candidate", "seed"}
                if kind == "nodes"
                else {"reviewed", "candidate"}
            )
            status = item.get("status")
            check(
                isinstance(status, str) and status in statuses,
                f"{where}.status: expected {sorted(statuses)}",
            )
            seed = status == "seed"
            verified = item.get("last_verified")
            if seed:
                check(
                    verified is None,
                    f"{where}.last_verified: seed must be null (research pending)",
                )
            else:
                try:
                    if not isinstance(verified, str) or not re.fullmatch(
                        r"\d{4}-\d{2}-\d{2}", verified
                    ):
                        raise ValueError
                    parsed = date.fromisoformat(verified)
                    check(parsed <= date.today(), f"{where}.last_verified: future date")
                except ValueError:
                    check(
                        False, f"{where}.last_verified: expected real YYYY-MM-DD date"
                    )
            sources = item.get("sources")
            if check(isinstance(sources, list), f"{where}.sources: expected array"):
                check(bool(sources) or seed, f"{where}.sources: evidence required")
                check(
                    not sources or not seed,
                    f"{where}.sources: seed is unverified; promote after research",
                )
                urls = set()
                for source in sources:
                    if not check(
                        isinstance(source, dict) and set(source) == {"label", "url"},
                        f"{where}.sources: expected label/url objects",
                    ):
                        continue
                    check(
                        text(source["label"], 120),
                        f"{where}.sources: label required (max 120 characters)",
                    )
                    url = source["url"]
                    if check(
                        safe_url(url),
                        f"{where}.sources: unsafe or invalid HTTP(S) URL: {url!r}",
                    ):
                        check(url not in urls, f"{where}.sources: duplicate URL")
                        urls.add(url)
            if kind == "nodes":
                for field in ["name", "summary", "participation"]:
                    check(
                        text(item.get(field), 120 if field == "name" else 2000),
                        f"{where}.{field}: nonempty text required within length limit",
                    )
                for field, catalog in [
                    ("region", "regions"),
                    ("category", "categories"),
                ]:
                    value = item.get(field)
                    check(
                        isinstance(value, str) and value in catalogs[catalog],
                        f"{where}.{field}: unknown {field}",
                    )
            else:
                check(
                    text(item.get("description")),
                    f"{where}.description: evidence explanation required",
                )
                value = item.get("type")
                check(
                    isinstance(value, str) and value in TYPES,
                    f"{where}.type: unknown relationship type",
                )

    nodes = {n["id"]: n for n in records["nodes"] if identifier(n.get("id"))}
    seen = set()
    for index, edge in enumerate(records["edges"]):
        where = f"edges[{index}]"
        source, target, kind = edge.get("source"), edge.get("target"), edge.get("type")
        if not check(
            identifier(source) and identifier(target), f"{where}: invalid endpoint ID"
        ):
            continue
        check(source != target, f"{where}: self-link not allowed")
        for endpoint in [source, target]:
            if check(endpoint in nodes, f"{where}: dangling endpoint {endpoint}"):
                check(
                    nodes[endpoint].get("status") != "seed",
                    f"{where}: research-pending seeds cannot have relationships",
                )
        if isinstance(kind, str):
            pair = (
                tuple(sorted([source, target]))
                if kind == "runs_activities_with"
                else (source, target)
            )
            key = (kind, *pair)
            check(key not in seen, f"{where}: duplicate relationship")
            seen.add(key)
    return errors


def unique_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "static/data/megamap.json",
    )
    args = parser.parse_args()
    try:
        data = json.loads(
            args.path.read_text(encoding="utf-8"), object_pairs_hook=unique_keys
        )
        errors = validate(data)
    except (OSError, ValueError) as error:
        errors = [str(error)]
    if errors:
        print("\n".join(f"ERROR: {error}" for error in errors), file=sys.stderr)
        return 1
    print(
        f'Megamap valid: {len(data["nodes"])} organizations, {len(data["edges"])} relationships'
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
