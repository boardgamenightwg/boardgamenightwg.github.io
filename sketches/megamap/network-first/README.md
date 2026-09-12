# Network-first: discover through adjacency

Disposable standalone HTML mockup for a proposed `/megamap` on boardgamenightwg.com. Not a production implementation or an actual site route.

## Design stance
A large conceptual network makes the ecosystem itself the primary interface; a dark contextual panel explains the selected node.

## Key choices
- Cream, ink, teal, and orange with system typography; no network assets or dependencies.
- Boston / Bay Area switch replaces the seed dataset and resets selection.
- Native buttons, visible keyboard focus, pressed states, and a live status announcement.
- Click organization names in connection details to follow a link to the other node.
- Proposed introductions and illustrative shared interests are explicitly distinguished. Confirmed relationships: **none verified**.
- Prominent placeholder disclaimer; repeated caveats in relationship detail. FabLab's precise identity remains TBD.
- “Propose an edit” and “Suggest a correction” open an explanatory, keyboard-accessible native dialog. Nothing is submitted or saved; Escape closes it.
- Desktop graph uses positioned native buttons over a decorative SVG edge layer. On small screens it becomes a readable node list with link counts, followed by the same connection detail panel. Coordinates are conceptual, not geographic.

## Trade-offs
- Strong at: noticing adjacency and exploring by clicking through the network.
- Weak at: dense datasets, linear scanning, and precise connection labeling without selecting a node. A production version would need scaling/layout research, not just more dots.
- Best for: exploratory discovery and a small curated ecosystem.

## Open
Open `/home/rosie/workspace/boardgamenightwg.github.io/sketches/megamap/network-first/index.html` directly in any modern browser, or use:

```sh
xdg-open /home/rosie/workspace/boardgamenightwg.github.io/sketches/megamap/network-first/index.html
```

No build step or server required. The small header navigation links to the sibling variant; all other functionality is inline and independent.

## Scope and verification
All descriptions, category assignments, and edges are illustrative placeholders, not verified research. Seed names were supplied for design exploration only. No actual collaborations are asserted. The future public PR process is explanatory, not connected to a repository editor. The noindex hint is not privacy or access control.

Inline JavaScript syntax is locally checked; visual browser testing and screenshots are delegated to the parent agent. No production files, data, or routes were modified.
