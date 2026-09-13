# Homepage event preview

## Approved design A

Griz selected **A, club nights + community preview**, in Discord thread
`1548402906708050071`, message `1548516891813879809` ("Let's do A").

The homepage keeps the existing header, logo, IBM Plex Sans, palette, light/dark
mode, 720px content width, heritage line and footer. Its page-specific layout is:

1. **Next club game nights:** compact Boston and Bay Area rows, in that order.
2. Newcomer link to the FAQ.
3. **Around the community:** up to three compact native disclosures, each with a
   decorative month/day tile, region/category eyebrow, title, and full schedule.
   Expanded rows retain organizer metadata, description, permitted links and a
   direct link to the corresponding community-page anchor.
4. **Browse all community events**, always available, then the heritage line.

The preview banner and illustrative fixed-snapshot events are not shipped. The
old 600px Luma calendar iframe is removed from the homepage, not from Luma.

## One maintained source

Continue maintaining only these canonical Markdown pages:

- `content/boston.md` and `content/bayarea.md`: next club date under `## When`,
  venue as the first bold text under `## Where`, and the chapter's full RSVP,
  access and other instructions.
- `content/community.md`: region `##` headings (Boston, Bay Area, Major Robotics
  Events), event `###` headings, category badge, and When / Where / Host prose.

`templates/index.html` uses Zola `get_page` to embed their generated HTML inside
**inert templates** at build time. No second event list, external API, Luma write,
extra preprocessing, backend or runtime dependency is needed. A normal Zola
build automatically propagates a source-only edit to the homepage.

The homepage module reads those templates, never instantiates them. It builds
fresh nodes with text and a small allowlist (`p`, `strong`, `b`, `em`, `span`,
`br`, `a`); source scripts, iframes, images, handlers, styles and other attributes
are not copied. Links are resolved against the canonical community URL and only
HTTP(S) destinations become active. No source string is assigned to `innerHTML`.
The community enhancement and shared theme/analytics code are unchanged.

## Date-only selection policy

The client evaluates the current date when the homepage opens. Date comparison
uses **America/New_York** for Boston and Major Events, and
**America/Los_Angeles** for Bay Area, independent of the visitor's timezone.
The date model accepts an injected `Date` for deterministic tests.

Supported dates use a known English full month or three-letter abbreviation,
a day, and a four-digit year. Same-month ranges such as `September 22–24, 2026`
and semicolon-separated secondary days such as
`September 23, 2026 @ 7 AM; September 24 @ 8 AM` are supported. A secondary day
may repeat the same year; otherwise it inherits the first date's year. Named
schedule clauses such as `; workshops: 8 AM` are preserved, not read as dates.
Impossible dates, reversed ranges, malformed years, numeric-only dates,
unknown schedules, cross-month secondary dates and unsupported syntax are not
guessed. Extend the model tests before supporting a new date grammar.

An event stays eligible **through its last listed calendar day**, including an
ongoing multi-day event. No exact start/end instant is inferred; even a night
ending at "midnight" is retained through its explicitly listed day, not given
an invented next-day timestamp. The complete organizer schedule remains visible
verbatim (Markdown presentation whitespace aside); date tiles are only a visual
aid, never a replacement schedule or fabricated machine timestamp.

- **Club:** a recognized same-day/future/ongoing date displays its full schedule,
  first bold venue name, and **Details / RSVP** linking to the chapter. Missing or
  expired dates display **Next date coming soon**. Nonempty unsupported schedules
  or unsupported When markup retain **Check the chapter for the latest date**.
- **Community:** select the earliest-starting eligible event in each region,
  then fill unused slots from the remaining eligible events. Sort the final
  selection by start date; source order breaks ties within a region. No more than
  three rows appear. Unknown/expired dates are not labeled upcoming. An empty
  selection gives a neutral directory signpost, not an assertion that no events
  exist.

This is a compact preview, not a complete calendar. The directory always remains
accessible, including dates omitted by this conservative grammar. Selection is
recomputed on page load; a tab left open across midnight needs a reload.

## Progressive startup and fallback

The route-specific module is **async after its source and destination DOM**.
It does not wait for DOMContentLoaded, Feather, or the inherited deferred
analytics queue. Moving it to the head or removing `async` would break that
startup guarantee. Do not change shared analytics to fix homepage startup.

Without JavaScript (or with the module blocked), chapter links, the community
directory, FAQ and heritage remain visible. Club copy says to check the chapter,
not "coming soon" before dates have actually been checked. No active iframe or
third-party event embed is required to use this fallback.

## Verification

With Zola 0.17.2, Node 22, and the existing browser test requirements installed:

```sh
python -m unittest discover -s tests -v
node --test tests/*.test.mjs
zola build
python -m pre_commit run --all-files
```

`test_homepage.py` builds both the real site and a temporary copy with edits only
to the three canonical Markdown sources. Fixture expectations are deterministic;
real-source checks don't assume a fixed event count or club empty state. The
suite checks source propagation, native keyboard disclosures, full metadata,
unsafe-link/active-content rejection, no-JS and blocked-script navigation,
320px/390px/desktop layouts, and the theme toggle. Its startup regression retains
the complete generated document and **holds the real analytics request pending**
while checking positive populated UI; aborting analytics is not equivalent.

The homepage date-model Node suite is invoked from Python unittest discovery, so
it runs in the existing browser CI job (which explicitly installs Node 22).
The dependency-free Megamap job stays scoped to `test_megamap.py`.

Local implementation checks passed: 38 Python tests, 12 Node tests, including the
new homepage browser/model coverage. These are local build/test results, **not a
claim of a merged, deployed or visually approved release**.

Before reporting a release as working:

1. Independently compare the built homepage to mockup A with the genuine site
   font/logo/icons, including 320px, 390px and dark mode.
2. Check actual current dates, open a disclosure, follow organizer/chapter links,
   and exercise the native keyboard controls and no-script fallback.
3. After the user approves merge, confirm the intended Pages deployment and
   repeat functional and visual checks on the actual deployed homepage.
4. Report local tests, visual comparison, merge and deployment separately. An
   HTTP 200, successful build or clean console alone is not proof of the design.
