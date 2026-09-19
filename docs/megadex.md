# Contributing to the experimental Megadex

The public `/megadex/` page is a rolling company index for the club's two
regions: **careers links** for robotics companies around Boston and the Bay
Area, plus recent sourced **news** and regional geographic maps. It is the
sibling of the experimental Megamap, but has no relationship model: companies
remain a flat, sourced directory, with optional verified locations.

The page is deliberately **public but unlisted**: no navigation link, no
sitemap entry, no search-index entry, and page-only `noindex,follow`. This is
not access control. The HTML, JSON and repository are public; never add
private contact information, private notes or secrets. Do not globally deindex
the site or block crawlers from reading the page's robots meta tag.

## A data-only pull request

1. Edit **`static/data/megadex.json`**. Do not edit templates, generated
   `public/` files or stylesheets to add a company or a news item.
2. **Every URL must be checked before you commit it.** `careers_url` must
   resolve (follow redirects, confirm it is the company's own job listings,
   not a third-party aggregator unless the company has no board of its own).
   Some careers pages block datacenter IPs (HTTP 403 from CI); verify in a
   real browser session and note that in the PR description if curl fails.
3. Company entries need: `id` (kebab-case), `name`, `region` (`boston` or
   `bay`), `website`, `careers_url`, a short `summary` of what they do,
   `news` (may be empty), and `last_verified` (ISO date, never in the future).
   Avoid repeating the town in the summary when it has a `location` field.
4. **News items** are `{date, headline, url}`, newest-first, one line each.
   Prefer primary sources (company blog, funding/product announcement,
   reputable outlet). No speculation, no rumor, no opinion. Aim to prune
   items older than ~3 months so the page stays a "recent news" list.
5. Run the validator and tests below. Include the URLs you checked and the
   actual verification results in the PR description. A human reviewer
   approves entries before merging.

### Entry criteria

- The company does robotics/automation work relevant to club members, and
  has a presence (HQ, office, or primarily-remote hiring) in Boston or the
  Bay Area.
- Venues that have hosted club game nights are welcome entries (e.g. Locus,
  Vecna); note that in the summary.
- Clubs, meetups and community groups belong on the Megamap, not here. When
  in doubt, ask in the epic issue rather than duplicating.

## Verified locations (optional)

A company without `location` remains valid and displays **Location not mapped**.
Never infer a coordinate from a company name or an unsourced summary. The page
performs no runtime geocoding and asks for no location permission.

`location` has exactly these fields (omit the entire object if unverified):

```json
{
  "label": "Waltham, MA",
  "lat": 42.3765,
  "lon": -71.2356,
  "precision": "city",
  "source_url": "https://example.org/contact",
  "verified": "2026-09-10"
}
```

This is a schema example, **not evidence for a real company**. Supply a primary
company source confirming its location. Coordinates must be finite JSON numbers
(not strings or booleans), latitude in [-90, 90], longitude in [-180, 180].
`label` is nonblank and at most 200 characters. `precision` is either `city`
(plainly labeled **approximate city pin**, never an office address) or `address`
(only with an explicitly sourced address and matching verified coordinates).
`source_url` must be a safe HTTPS URL; `verified` must be a real YYYY-MM-DD date,
not in the future. The validator checks structure, not whether the source proves
the claim: human review must check the source and geocode. Record research in
[location evidence](megadex-locations.md). Preserve 2-space, sorted-key JSON.

## Map implementation and fallbacks

- The list is server-rendered, compact, and always usable. Jobs are the primary
  action; news uses native disclosure rows, including without JavaScript.
- Each region has an interactive Leaflet 1.9.4 map, beside the list on desktop
  and above it on mobile. Numbered markers match the list's numbered badges.
  Equal coordinates share a marker and popup listing all companies and jobs:
  no made-up geographic offsets. “Show on map” selects the matching company.
- Vendored Leaflet JS, CSS, and license live in `static/vendor/leaflet/`. The
  page-only async module initializes independently of deferred analytics. Popup
  company text is created through safe DOM nodes, never HTML concatenation.
- OpenStreetMap raster tiles use `https://tile.openstreetmap.org/{z}/{x}/{y}.png`
  with visible attribution, ordinary browser caching and Referer. No API keys,
  prefetch, offline downloads or geocoding. Follow the
  [OSM tile policy](https://operations.osmfoundation.org/policies/tiles/).
- Scroll-wheel zoom is disabled. Keyboard users can activate pins with Enter
  or Space; list map buttons move focus into the selected popup, and Escape or
  popup close returns focus to the invoking button. Resizing refits verified
  locations. A shared pin keeps every company discoverable.
- No JavaScript or missing Leaflet leaves the list and external OpenStreetMap
  links usable. Failed tiles show a visible **Map unavailable** warning, not a
  misleading success state. Stalled tiles get the same warning after 12 seconds.

## Validation and tests

```bash
python scripts/validate_megadex.py
python -m unittest discover -s tests -p 'test_megadex.py' -v
zola build --base-url http://127.0.0.1:8767   # pinned 0.17.2 in CI
# Requires tests/requirements-browser.txt and Playwright Chromium:
python tests/megadex_smoke.py
```

The validator enforces strict fields, safe URLs, dates, coordinate ranges and
newest-first news. Schema regressions cover valid/missing locations, booleans,
NaN/infinities, ranges, unknown precision and invalid metadata.

CI uses **real vendored Leaflet with intercepted synthetic tile images**; it
never calls the public tile service. Source-derived assertions avoid fixed
company counts. A separately built synthetic fixture covers co-located pins,
unmapped/empty regions, news, keyboard and mouse selection, safe popups,
responsive bounds at 1440/390/320px and initial mobile, dark-theme readability,
map-button focus, tile/library/no-JS failure, and genuinely pending analytics.
Fallback screenshots (clearly named `megadex-fixture-*`) are written under
`build/megadex-screenshots/` and uploaded by CI; they are not live-map evidence.
The temporary fixture never changes the canonical data file.

For bounded manual network QA, `python tests/megadex_smoke.py --live-tiles`
additionally checks real OSM responses and saves `megadex-live-*` screenshots
from the actual source data. Do not enable live-tile QA in repeated CI runs;
verify actual tile imagery, marker bounds, attribution, mobile and dark mode
before calling a screenshot a working map.
