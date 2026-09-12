# boardgamenightwg.com

To build locally [install zola](https://www.getzola.org/documentation/getting-started/installation/) then run:

```bash
zola serve
```

## Community Events

Keep editing `content/community.md` using `.claude/skills/community-event/SKILL.md`.
The Markdown remains the only event source; there is no additional feed, backend,
or generated event data to maintain. This UI does not add, expire, sort, or sync
events. Existing approval, chronological ordering, expiration, and Luma sync rules
still apply.

The community-only template progressively enhances the rendered chapters into a
compact agenda. Boston is the default; visitors can choose Bay Area, Major Events,
or All regions, search any event text (including the organizer's date), and filter
by the source badges. Reset clears search/type filters within the selected region.
Native disclosure rows retain the full details and canonical RSVP links. Dates are
shown verbatim, with no date parsing, inferred timezone, or shortened ranges.

Existing chapter/event fragments reveal their chapter and expand the targeted
event. The submission anchor remains outside filtering. Region buttons do not
rewrite browser history or require localStorage, location permissions, or cookies.
Without JavaScript, with a missing script, or with unsupported source structure,
the original full Markdown remains readable. An unrecognized chapter, missing
When/Where/Host labels, or an interactive event heading conservatively skips the
whole enhancement; update the adapter/tests if the content format changes.

### Browser regression tests

Requires Zola 0.17.2 (the deployment version), Python 3.11, and Chromium:

```bash
python3 -m venv /tmp/bgnwg-community-tests
/tmp/bgnwg-community-tests/bin/python -m pip install -r tests/requirements.txt
/tmp/bgnwg-community-tests/bin/python -m playwright install --with-deps chromium
/tmp/bgnwg-community-tests/bin/python -m unittest discover -s tests -v
```

The suite builds Zola to a temporary directory and serves on a free local port.
It compares the actual rendered source before/after enhancement dynamically, so
routine event cleanup does not require updating counts. A clearly synthetic HTML
fixture covers fixed counts, long titles, full date ranges, unknown times, gated
venues, empty chapters/results, malformed markup, plain-text safety, failure
fallback, keyboard behavior, fragments, and 1200/390/320px light/dark layouts.
Tests isolate the community code from unrelated base-template scripts and remote
fonts; they require no third-party network services after dependency installation.
GitHub Actions runs this suite on PRs and main, separately from formatting.

For integration/visual QA, also open the **unmodified** Zola page with normal site
scripts enabled: verify the robot image and IBM Plex Sans actually load, toggle
the existing dark-mode button, try the filters and keyboard disclosures, check
320px and desktop widths, then disable JavaScript and confirm all original events
and the submission email remain accessible. Do not commit LFS pointer replacement
artifacts or test screenshots.
