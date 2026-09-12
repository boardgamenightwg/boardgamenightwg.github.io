# Contributing to the experimental Megamap

The public `/megamap/` page explores communities, participation and specific
sourced relationships. **The relationship model is experimental, not final.**
A label is an interpretation of cited evidence, not a blanket partnership,
endorsement, guaranteed public access or a claim that other connections do not
exist. Discuss scope/model changes in [epic #96](https://github.com/boardgamenightwg/boardgamenightwg.github.io/issues/96).

The page is deliberately **public but unlisted**: no navigation link, no sitemap
entry, no search-index entry, and page-only `noindex,follow`. This is not access
control. The HTML, JSON and repository are public; never add private contact
information, private notes or secrets. Do not globally deindex the site or block
crawlers from reading the page's robots meta tag.

## A data-only pull request

1. Edit **`static/data/megamap.json`**. Do not edit JavaScript, templates, generated
   `public/` files or coordinates to add an organization or connection.
2. Cite public primary sources. Read what they say about **community participation**,
   not just a company's products or a one-time BGNWG venue. Do not infer that
   hosting one game night means an ongoing affiliation or public drop-in access.
3. Keep summaries concise; distinguish public events, membership, program-specific
   access and student-centered communities. Confirm current announcements rather
   than perpetuating stale location/frequency claims.
4. Add relationships only when supported. An organization with no edges is useful.
   Prefer stable event/announcement URLs over rolling calendars, and describe
   the specific event/date instead of implying a recurring commitment.
5. Run the validator and tests below. Include sources, relevant quotes, scope
   questions and the actual verification results in the PR description. A human
   reviewer approves the evidence and interpretation before merging. No automatic
   posting or deployment is performed by the page.

### Copyable research placeholder

For an **unverified suggestion**, copy this object into `nodes`, replacing the
example ID and name. This is a schema example, not an actual organization. Do not
connect a seed to other nodes. After checking primary sources, replace the
research-pending text, add source links and the verification date, and change its
status to `reviewed` or `candidate`.

```json
{
  "category": "robotics",
  "id": "example-robot-club",
  "last_verified": null,
  "name": "Example Robot Club",
  "participation": "Research pending. Participation and access have not been verified.",
  "region": "boston",
  "sources": [],
  "status": "seed",
  "summary": "Research pending. This suggested community has not been verified."
}
```

For a sourced relationship, copy an existing record in `edges`, give it a new
stable ID, use the actual endpoint IDs, and replace the explanation, evidence and
date. Do not reuse an existing claim or citation for an unrelated pair of groups.

## Data format (version 1)

The root has exactly `version`, `regions`, `categories`, `nodes`, and `edges`.
`regions` and `categories` are nonempty ID-to-display-label objects. Existing IDs
include `boston`, `bay`, `social-play`, `robotics`, and `making`. Add a category or
region in these objects if needed; its control is generated automatically.
`all` is reserved for the all-categories filter.

Organization fields (all required):

| Field | Meaning |
| --- | --- |
| `id` | Stable lowercase hyphenated ID, beginning with a letter; at most 80 characters. Keep it when the name changes. |
| `name` | Public display name, nonempty, at most 120 characters. |
| `region`, `category` | IDs present in the root catalogs. One primary region/category per node. |
| `summary` | What this community does, nonempty plain text, at most 2,000 characters. |
| `participation` | How someone can take part, including access restrictions; same text limit. |
| `sources` | Array of `{ "label": "About / joining", "url": "https://…" }` objects. Labels are nonempty and at most 120 characters. |
| `last_verified` | Real, non-future `YYYY-MM-DD` date when the cited claims were checked; `null` for seeds. |
| `status` | `reviewed`, `candidate`, or `seed`, as described below. |

Statuses do not hide nodes:
- **reviewed**: source-backed profile reviewed for this experiment; not an endorsement.
- **candidate**: sourced, but community scope/inclusion is still a question (initial example: BU RASTIC).
- **seed**: explicitly unverified research placeholder. Say “Research pending” in
  summary and participation, use `sources: []` and `last_verified: null`. Seeds
  cannot have edges. Promote after research instead of filling in guesses.

Both reviewed and candidate records require at least one source and a verification
date. As initially implemented, Boston has six nodes and four sourced edges; the
Bay Area has three reviewed profiles and no recorded edges. Do not preserve these
counts artificially: tests derive visible counts from the current canonical data.

Every relationship requires exactly these fields:
`id`, `source`, `target`, `type`, `description`, `sources`, `last_verified`, `status`.
`source` and `target` are organization IDs, not URLs. `description` explains the
actual evidence and any limits (nonempty, at most 2,000 characters). Source objects
and dates use the same format as nodes. Edge status is `reviewed` or `candidate`,
never `seed`.

| Type | Meaning and direction |
| --- | --- |
| `shares_events_from` | Directed: the group listing an event → the group whose event it lists. The target's details correctly say **Events shared by ←**. One listing is not a recurring commitment. |
| `runs_activities_with` | Symmetric: A ↔ B. Enter it once, not in both directions. Use a description that explains the documented joint activity and provisional classification. |
| `operates` | Directed: an explicitly documented operator → the community/program it operates. Incoming label: **Operated by ←**. No initial example is asserted; do not invent one for completeness. |

The MassRobotics ↔ Women in Robotics Boston edge maps the source's statement about
joining communities through networking, education and mentoring. That mapping is
explicitly provisional; it does **not** assert that MassRobotics operates the group.
Cross-region edges are allowed if sourced. Following one switches region and clears
filters to reveal its target; the graph only draws edges whose endpoints are visible.

The validator rejects unknown fields/types/categories/regions, malformed JSON and
duplicate JSON keys, repeated node/edge IDs, duplicate relationships (including
reversed symmetric duplicates), dangling endpoints, self-links, invalid dates,
empty required text/evidence, and unsafe URLs. Source URLs must be absolute HTTP(S)
with a host, no embedded credentials, whitespace/control characters or backslashes.
Use HTTPS when available. URL syntax checks do not prove a source is accurate or
still online: that remains a human review responsibility.

## Local checks

Requirements: **Zola 0.17.2**, **Python 3.11+**, **Node 22**. No npm packages,
frontend compilation, framework, backend or Python package is needed for data and
unit checks. Install Git LFS and retrieve tracked image assets when cloning the site:

```sh
git lfs install
git lfs pull
python3 scripts/validate_megamap.py
python3 -m unittest discover -s tests -p 'test_*.py' -v
node --test tests/megamap-model.test.mjs
zola build
```

Always validate **before** Zola renders the JSON into the static fallback. Validation
also gates the existing deployment build. The new `Megamap checks` workflow runs
validation, unit tests, a pinned Zola build and a real headless Chromium smoke on PRs.
The existing pre-commit workflow is unchanged. Actual remote CI status must be checked
on the PR; local passes do not claim a GitHub Actions run happened.

### Optional real browser smoke

Use a project virtual environment (works on PEP 668 systems, no global pip install):

```sh
python3 -m venv .venv-megamap
.venv-megamap/bin/python -m pip install -r tests/requirements-browser.txt
.venv-megamap/bin/python -m playwright install chromium
# On a minimal Linux runner, provision browser system libraries if needed:
# .venv-megamap/bin/python -m playwright install --with-deps chromium
python3 scripts/validate_megamap.py
zola build --base-url http://127.0.0.1:8766
.venv-megamap/bin/python tests/megamap_smoke.py
```

Do not start another server on port **8766**: the test owns a temporary local HTTP
server and closes it afterward. It reads the real built site, not a mock HTML page.
It checks desktop grouping, AND filtering, no-results recovery, directional sourced
details, keyboard selection, hidden-node following, region switches, theme persistence,
mobile list/selection focus/back navigation, no-JS parity, fetch failure, literal
hostile text/URL rejection, page-only noindex and sitemap/nav exclusion. It captures
`/tmp/megamap-desktop.png` and `/tmp/megamap-mobile.png`.

The inherited analytics endpoint is blocked in normal smoke scenarios to avoid
waiting on unrelated tracking. A separate regression holds it **pending** and
requires the map to populate anyway. The page's async module is after its DOM,
so it cannot wait behind the inherited deferred analytics script. Shared font/icon
CDNs remain the site's existing dependencies; network failures there may affect
visual/theme tests. No external organization source is fetched by the map or tests.

## Implementation notes

- `templates/megamap.html` uses Zola `load_data` on the same canonical JSON for a
  complete server-rendered directory. It stays readable without JS or if fetch fails;
  successful enhancement collapses it rather than removing it.
- `static/scripts/megamap-model.mjs` contains pure filtering, state, direction,
  layout and URL helpers. `megamap.mjs` creates DOM/SVG elements and uses `textContent`
  for all dataset text. There is no interpolated inline JSON or data-bearing `innerHTML`.
- The two-column layout is deterministic in data order and grows vertically. No
  manual positioning is needed. It is a browsing network, not geography; large
  datasets may need a different layout later.
- Search and category combine with the selected region. Changing region resets
  search/category; filtering reconciles selection. Status updates are polite and
  debounced. Typing never moves focus. Explicit node selection on stacked layouts
  reveals/focuses details; explicit relationship-following reveals its target.
- CSS is scoped to the page, inheriting the site's font and light/dark variables.
  Base-template changes only add extension blocks for page head/body styling.
- `templates/sitemap.xml` retains Zola 0.17.2's builtin entry/lastmod behavior except
  `/megamap/`. If this route changes, update the exclusion and smoke assertion together.
