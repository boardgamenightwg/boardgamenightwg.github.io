# Community Events: design and release checks

`content/community.md` remains the only maintained event source. The page-specific
`community.html` template loads the stylesheet and a progressive DOM enhancement.
Without JavaScript, or when the source structure is unsupported, the full original
content remains readable.

## Approved design

The selected chapter-first mockup and implementation authorization are recorded in
[issue #97](https://github.com/boardgamenightwg/boardgamenightwg.github.io/issues/97).
The production rendering should retain the site's logo, header, IBM Plex Sans,
720px content width, category colors, and light/dark theme, with:

- Region navigation in a narrow desktop sidebar, responsive on small screens.
- The selected region's heading and description above search and type filters.
- Month/year headings, a prominent month/day column, and compact disclosure rows.
- Region/category labels above each event title, and its full original schedule.
- Organizer links, descriptions, event anchors, and the submission path preserved.

Mockup-only preview labels and fixed-snapshot event counts are not production UI.
Date tiles are display aids, not replacements for the full source date/time.

## Startup must not depend on analytics

The inherited base template includes an unrelated **deferred** analytics script.
If that request stays pending, subsequent deferred scripts wait behind it even
when their downloads have completed. The result can look like the old long list
with no JavaScript exception.

The community script is loaded **async after the event DOM**. It has no dependency
on the analytics script, Feather, or DOMContentLoaded. Keep it after the source
markup; moving it into the head would invalidate that assumption. Do not remove
or reconfigure shared analytics as part of a community-page fix.

## Tests

Install the dependencies from `tests/requirements.txt`, Chromium via Playwright,
and Zola 0.17.2, then run:

```sh
python -m unittest discover -s tests -v
```

Most browser cases isolate the enhancement from external services. The separate
pending-analytics regression retains the complete generated document and holds
the real analytics request pending. It must still render usable region controls,
search, and event rows before the request finishes. Immediately aborting analytics
is **not** equivalent: it releases the deferred script queue and hides this bug.

Design tests check the actual event source, date grouping and fallback, and key
layout relationships. Content-preservation, keyboard/fragment navigation,
no-JavaScript, malformed-source, and mobile/theme tests remain required.

## Before reporting a release as working

1. Confirm the intended commit's Pages deployment succeeded.
2. Open the actual deployed `/community/` page. Verify the positive enhanced state,
   not just a successful HTML/CSS/JS download or absence of console errors.
3. Compare a desktop screenshot with the approved chapter-first mockup: heading
   placement, month headings, date column, sidebar, and compact rows.
4. Exercise region selection, search, type filtering, reset, and a disclosure.
5. Check desktop, 390px, and 320px layouts and the actual theme toggle.
6. Confirm all current event details and organizer links are accessible, including
   with JavaScript disabled.

Report merge, deployment, functional checks, and visual comparison separately.
A local preview is not proof of what users see on the deployed site.
