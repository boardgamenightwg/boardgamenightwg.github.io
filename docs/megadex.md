# Contributing to the experimental Megadex

The public `/megadex/` page is a robotics-company index for both club regions:
**careers links**, recent sourced **news**, and regional geographic maps. It is
not a directory of all board-game venues or host organizations. It is the sibling
of the experimental Megamap, but companies remain a flat, sourced directory with
optional verified locations.

The page is deliberately **public but unlisted**: no navigation link, no
sitemap entry, no search-index entry, and page-only `noindex,follow`. This is
not access control. The HTML, JSON and repository are public; never add
private contact information, private notes or secrets. Do not globally deindex
the site or block crawlers from reading the page's robots meta tag.

## A data-only pull request

1. Edit **`static/data/megadex.json`**. Do not edit templates, generated
   `public/` files or stylesheets to add a company or a news item.
2. **Every URL must be checked before you commit it.** A non-null `careers_url`
   must be an official employer careers page, a job board linked by that
   employer, or a verified employer-maintained hiring profile. Follow redirects and verify identity, not just HTTP status.
   Use explicit `null` when no public employer jobs page can be verified;
   the page says **Jobs page not listed**, not that the organization is not
   hiring. Do not substitute a generic homepage or ecosystem job board.
   Some careers pages block datacenter IPs (HTTP 403 from CI); verify the
   employer's link in a browser and document access limitations. Parent-employer
   careers pages for labs/centers must be clearly described as employer-wide.
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
- Hosting a club event supplies a candidate, not an exception to the robotics-
  company scope. Exclude breweries, general makerspaces, university centers,
  and industry associations/hubs that are not robotics companies. Deduplicate
  renamed companies and branded labs/venues under the responsible employer.
- **Willow Garage is an explicitly approved historical Easter egg.** Keep its
  closed/historical labeling and safe club-history link; never give it a current
  office pin or hiring link.
- Community groups and non-company venues belong on the Megamap or event pages,
  not here. [Host research](megadex-hosts.md) records both included candidates and
  scope exclusions. Past venue addresses prove event history, not a current
  office; current map locations need their own source.
- Companies need not have hosted the club. Additional robotics companies can be
  included when public sources verify their identity, regional presence and
  careers destination (or an honest missing-link state). See
  [additional company sources](megadex-company-sources.md). Private mail may
  help identify candidates, but never publish correspondence, personal contacts,
  or inferred hosting/relationship claims in this public directory.

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
- Boston and Bay Area tabs show one region at a time, defaulting to Boston,
  using the Community page's chapter-selector colors and styling. Arrow keys,
  Home and End switch tabs with keyboard focus. Without JavaScript both lists
  remain visible; tabs still work if the optional map library fails.
- Maps initialize when their tab is first selected; revisiting a tab refits the
  existing map without duplicate markers or hidden-container sizing errors.
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
unmapped/empty regions, region-tab switching and keyboard navigation, news,
keyboard and mouse selection, safe popups,
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
