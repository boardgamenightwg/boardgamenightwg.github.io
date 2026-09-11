# Mockup verification

## Relationship prototype

- Chromium checks pass at 1440/768/390px: six Boston nodes and four sourced edges, correct incoming/outgoing wording, evidence links, candidate and unconnected-node explanations, category/search/empty/reset, hidden-connection navigation, keyboard selection, modal/Escape and both themes.
- Bay Area remains three seed names with no fabricated connections and an explicit research-pending notice.
- No uncaught JavaScript errors or horizontal overflow; no overlapping graph nodes at desktop/tablet widths.
- Desktop and mobile screenshots visually inspected. Arrow endpoints sit outside fixed-height cards; symmetric community-activity edge has arrowheads at both ends.
- This is a throwaway model experiment, not an approved production schema or comprehensive accessibility audit.

## Site-themed hybrid

- Live CSS fetched and compared byte-for-byte with the repository stylesheet before implementation; live site visually inspected.
- Chromium checks at 1440/768/390px: area switching, combined search/category filters, filtered edge counts, selection consistency, following hidden connections (filters reset), empty state/reset, keyboard selection and contribution modal/Escape.
- Light/dark rendering and exact background colors verified at each width; dark preference survives reload.
- No horizontal overflow or uncaught JavaScript errors in these checks.
- Light desktop, dark desktop and light mobile screenshots visually reviewed. Same live robot logo is embedded in the mockup.
- The hybrid preserves the 720px header/intro and expands only the desktop map to 1060px. Mobile uses a compact network list.

## Original variants

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
