# Mockup verification

Both variants were exercised in Chromium at 1440px, 768px, and 390px widths.

- Region changes show four Boston seeds and three Bay Area seeds.
- Organization selection updates the detail heading.
- Connection buttons navigate to the linked organization and focus its heading.
- Keyboard Enter activates organization buttons.
- Contribution explanation opens as a modal and closes with Escape.
- Directory search, category filters, and no-results reset work.
- No horizontal document overflow at any tested width.
- No uncaught JavaScript errors in these runs.
- Desktop and 390px screenshots visually reviewed; mobile uses stacked content, with a compact node list instead of a cramped graph. Mobile selections focus the detail heading.

The existing Zola build passes. These sketch files are not production content and do not create a /megamap route. This is a scoped interaction smoke check, not a full accessibility audit or production test suite.
