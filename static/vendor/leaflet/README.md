# Leaflet 1.9.4 (vendored)

Upstream: https://leafletjs.com/ and https://github.com/Leaflet/Leaflet/tree/v1.9.4
License: BSD-2-Clause, reproduced in `LICENSE`.

Downloaded from the pinned npm package via:

- https://unpkg.com/leaflet@1.9.4/dist/leaflet.js
- https://unpkg.com/leaflet@1.9.4/dist/leaflet.css
- https://unpkg.com/leaflet@1.9.4/LICENSE
- `https://unpkg.com/leaflet@1.9.4/dist/images/{layers.png,layers-2x.png,marker-icon.png}`

The three images cover the CSS references. Robodex uses its own DOM-based
numbered div icons, not the standard marker icons. No third-party CDN request
is needed at runtime. Repository formatting normalizes text files to LF line endings and ensures
a final newline; otherwise these are upstream distribution files. Tiles are fetched separately
from OpenStreetMap; see `docs/robodex.md` for attribution and tile-policy rules.
