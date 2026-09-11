# Megamap: design exploration

Tracking epic: https://github.com/boardgamenightwg/boardgamenightwg.github.io/issues/96

Two disposable, clickable directions for a future unlisted `/megamap` ecosystem explorer. These files are outside Zola's content/static directories and do not publish the route or change the live site.

- [Network-first](network-first/index.html): spatial relationships first, selected organization details alongside.
- [Directory-first](directory-first/index.html): browse organizations first, inspect role and relationships on selection.

## Try them

Download this folder and open either `index.html` in a browser. Each mockup contains its own styles, scripts, and sample data, with no external dependencies. Or serve the repository with `python3 -m http.server 8766` and open `http://localhost:8766/sketches/megamap/network-first/` (or `directory-first/`).

Try switching regions, selecting different organizations, the contribution explanation, and narrowing the browser to a phone width. Filters/search are exploratory UI, not locked product requirements.

## Desktop previews

### Network-first
![Network-first desktop mockup](previews/network-first-1440.png)

### Directory-first
![Directory-first desktop mockup](previews/directory-first-1440.png)

Mobile previews: [network](previews/network-first-390.png) · [directory](previews/directory-first-390.png).

## Content warning

Organization names come from Griz's initial seed list. Descriptions, categories, and links in these mockups are illustrative design fixtures, not researched claims or confirmed partnerships. The specific Boston FabLab is unresolved. The production dataset needs canonical sources and verified relationship semantics.

## Review questions

1. Should the first screen emphasize connections (network) or finding an organization (directory)?
2. Does the detail panel explain enough about an organization's role, participation, and resources?
3. Should shared interests be visible as lines at all, or should lines be reserved for documented relationships?

No production implementation until Griz selects a direction or requests a hybrid.
