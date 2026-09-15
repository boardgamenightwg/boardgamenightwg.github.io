# Site-themed hybrid: network + filters

This iteration follows Griz's selection: A's network-first layout with B's search and interest filters, themed to the current boardgamenightwg.com website.

## Design stance
Make Megamap look like part of the club website, not a separate product.

## Theme grounding
- Live `/styles/main.css` was fetched and matched the repository stylesheet.
- Uses IBM Plex Sans 400/700, the existing robot logo, club header/navigation and footer.
- Exact site colors: white/#444/#222 with #3273dc links; dark #333/#ddd/#eee with #8cc2dd links.
- Reuses orange/blue/purple badge colors and blue note treatment.
- Header and introduction keep the 720px reading column. The map alone widens to 1060px on desktop to fit network and details side by side; it stacks on smaller screens.
- No Megamap navigation link is added. This remains an unlisted design sketch, not a deployed route.

## Try it
Open `index.html` in a browser. Styles, sample data, interactions, and a resized copy of the live logo are embedded. IBM Plex Sans loads from Google Fonts, as on the live site; offline it falls back to Verdana/sans-serif. Navigation links lead to the existing live pages.

Switch area, search a name or interest, filter by category, select nodes and follow connections. Search/category filters apply together to the graph; only edges between visible nodes remain. A hidden selection switches to the first match, or no selection in the empty state. Following a connection clears filters only when its target is hidden.

The sun/moon button switches light/dark themes and remembers the choice with a mockup-specific localStorage key. On phones, the network becomes a compact node list; selecting a node brings its details into view. Contribution buttons only explain the future PR flow.

## Trade-offs
- Strong at: matching the club identity while preserving network exploration and useful filtering.
- Weak at: this tiny sample cannot demonstrate large-graph density; the map is wider than existing reading pages; mobile favors readability over a spatial diagram.
- The approved direction is a layout choice, not approval of a data schema or production implementation.

## Data caveat
All descriptions, categories and connections are illustrative fixtures, not researched relationships. FabLab identity remains TBD. No verified affiliations are asserted.
