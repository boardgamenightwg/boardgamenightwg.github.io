# Contributing to the experimental Megadex

The public `/megadex/` page is a rolling company index for the club's two
regions: **careers links** for robotics companies around Boston and the Bay
Area, plus recent sourced **news**. It is the sibling of the experimental
Megamap; unlike the map it has no relationship model, just a flat, sourced
list.

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
   `bay`), `website`, `careers_url`, a one-or-two sentence `summary` (what
   they do + HQ town), `news` (may be empty), and `last_verified` (ISO date,
   never in the future).
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

## Validation and tests

```bash
python scripts/validate_megadex.py
python -m unittest discover -s tests -p 'test_megadex.py' -v
zola build   # pinned 0.17.2 in CI
```

The validator enforces the schema: field sets, kebab-case ids, known regions,
safe http(s) URLs without credentials, ISO dates not in the future, and
newest-first news ordering. CI also builds the site and runs a headless
browser smoke test that checks the rendered page.
