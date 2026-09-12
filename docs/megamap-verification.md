# Megamap pre-deployment verification

Local verification snapshot, September 12, 2026. This records checks of the real
Zola implementation, not a deployment claim. The production PR's Checks tab is
the source for remote CI status; the live route must be checked after review and
merge.

## Executed checks

```sh
python3 scripts/validate_megamap.py
python3 -m unittest discover -s tests -p 'test_*.py' -v
node --test tests/megamap-model.test.mjs
zola build --base-url http://127.0.0.1:8766
.venv-megamap/bin/python tests/megamap_smoke.py
pre-commit run --all-files
# Rebuild with the production URL before publishing:
zola build --output-dir /tmp/megamap-production-final
```

Use the virtual-environment setup in [the contribution guide](megamap.md).
The smoke suite owns port 8766. Use `--force` only when intentionally replacing
an existing generated Zola output directory.

Observed results:

- Validator: **9 organizations, 4 relationships valid**.
- Python: **six test methods passed**, including malformed-record mutations,
  duplicate JSON keys, seed restrictions, and relationship validation.
- Node: **seven tests passed** for filtering, selection, relationship navigation,
  edge direction, safe URLs, and automatic layout.
- Zola 0.17.2: **eight pages built**, including `/megamap/`.
- Real Chromium smoke: **passed** desktop grouping, combined filters, no-results
  recovery, keyboard selection, source-linked incoming/outgoing details, hidden
  target navigation, both regions, theme persistence, mobile selection/back
  navigation, no-JS fallback, fetch failure, literal hostile text/unsafe URL
  rejection, and unlisted-page behavior.
- The copyable contributor example was appended to an in-memory copy of the real
  dataset and passed validation. It is not part of the published organization list.
- **All existing pre-commit hooks passed**; `git diff --check` was clean.

## Independent visual and integration checks

A separate production-style build was served locally and exercised independently:

- Every profile and its relationship count matched the canonical JSON.
- Widths **320, 390, 600, 601, 768, 1000, 1001, 1280, and 1440px** had no horizontal
  page overflow, overlapping visible cards, or clipped card contents for the seed
  dataset.
- Mobile selection brought the selected heading into view. The return link was
  visible. Light/dark desktop and mobile detail screenshots were inspected.
- Dark screenshots waited for the inherited theme transition to finish rather
  than capturing intermediate colors.
- There were **no page errors or console errors during this independent run**.
- Generated sitemap URLs exactly matched the pre-change site's sitemap. Megamap
  is absent; the homepage, chapter pages, FAQ and community page contain neither
  a Megamap link nor a new noindex directive.

## Regressions covered

The initial real-browser load exposed an inherited deferred analytics request
holding up the new module: the directory appeared, but the interactive map stayed
hidden. The page-specific module now loads asynchronously after its DOM. A
committed regression holds that analytics request pending and requires the map to
populate anyway; normal smoke cases block unrelated analytics.

Other browser checks cover finite SVG coordinates on initial render, visible
symmetric arrowheads, mobile detail focus, and readable fallback content when
enhancement cannot run.

## Boundaries

- Source accuracy, inclusion decisions and provisional relationship interpretations
  still need human review. There are no initial Bay Area edges or `operates`
  examples. Reviewed is not an endorsement or a guarantee of public access.
- Unlisted means **public, not authenticated**. The JSON and repository are public.
- Existing site font/icon CDNs remain dependencies. No external organization API
  is fetched by the explorer or its normal browser tests.
- The deterministic layout grows vertically and does not require manual positions.
  A much larger dataset may warrant a different layout; it is not a geographic map.
