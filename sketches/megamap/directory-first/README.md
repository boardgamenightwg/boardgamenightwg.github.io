# Directory-first: understand before connecting

Disposable standalone HTML mockup for a proposed `/megamap` on boardgamenightwg.com. Not a production implementation or an actual site route.

## Design stance
An editorial reading experience puts organization names and understandable roles ahead of topology; cards, search, and interest filters lead into contextual connection notes.

## Key choices
- Cream, ink, teal, and orange with system typography; no network assets or dependencies.
- Boston / Bay Area switch replaces the seed dataset and resets selection.
- Native buttons, visible keyboard focus, pressed states, and a live status announcement.
- Click organization names in connection details to follow a link to the other node.
- Proposed introductions and illustrative shared interests are explicitly distinguished. Confirmed relationships: **none verified**.
- Prominent placeholder disclaimer; repeated caveats in relationship detail. FabLab's precise identity remains TBD.
- “Propose an edit” and “Suggest a correction” open an explanatory, keyboard-accessible native dialog. Nothing is submitted or saved; Escape closes it.
- Search matches names, categories, and short descriptions; category filters combine with search. Empty results offer a reset. Selection details intentionally remain visible even if filters exclude the selected card. Region changes clear search and filters.
- Mobile cards and readable connection lists provide a graph-free alternative without losing access to any node or edge.

## Trade-offs
- Strong at: scanning, searching, accessibility, and evaluating individual organizations.
- Weak at: seeing cross-ecosystem topology at a glance; connections require selecting an organization.
- Best for: first-time visitors looking for a relevant community and contributors checking individual listings.

## Open
Open `/home/rosie/workspace/boardgamenightwg.github.io/sketches/megamap/directory-first/index.html` directly in any modern browser, or use:

```sh
xdg-open /home/rosie/workspace/boardgamenightwg.github.io/sketches/megamap/directory-first/index.html
```

No build step or server required. The small header navigation links to the sibling variant; all other functionality is inline and independent.

## Scope and verification
All descriptions, category assignments, and edges are illustrative placeholders, not verified research. Seed names were supplied for design exploration only. No actual collaborations are asserted. The future public PR process is explanatory, not connected to a repository editor. The noindex hint is not privacy or access control.

Inline JavaScript syntax is locally checked; visual browser testing and screenshots are delegated to the parent agent. No production files, data, or routes were modified.
