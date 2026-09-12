# Relationship prototype — provisional

## Question
Do these three connection meanings help explain the relationships between social communities, or should the model change? The approved visual design is retained; this is a data/relationship experiment, not a new round of layout variants.

**Status:** Griz accepted these types for a prototype but explicitly said they might not be the answer. None is a finalized domain decision. No production route is introduced.

## Try it
Open `index.html` directly, or from the repository root run `python3 -m http.server 8766` and visit `/sketches/megamap/relationship-prototype/`. Google Fonts provides the same typography as the site; local fonts are the offline fallback. Other content is embedded. State is in memory; no submissions or data persistence.

## Nodes
Boston contains BGNWG, MassRobotics, Boston Robot Hackers, Women in Robotics Boston, Fab Hub Kendall (the Cambridge FabLab from the research), and **BU RASTIC as a candidate university-community inclusion**. RASTIC is present for its student-centered seminars/workshops/project community, not merely because it hosted a game night. Its card explicitly qualifies access.

Bay Area keeps the original seed names only; profiles remain unresearched and no invented edges are drawn.

## Edges under test
- **Shares events from →**: outgoing from the calendar/publication to the group whose events it lists. BGNWG → BRH; BGNWG → MassRobotics; MassRobotics → BRH.
- **Runs activities with ↔**: symmetric. MassRobotics ↔ Women in Robotics Boston uses the public statement that their communities have joined for networking/education/mentoring. The detail explains the literal evidence and that the classification is provisional.
- **Operates →**: available meaning, but there is no supported example among these nodes yet. No edge is invented just to demonstrate it.

Every displayed connection has its own explanation and evidence link. Incoming event-sharing lines read **Events shared by**, avoiding direction reversal. Fab Hub Kendall and RASTIC have no supported edge recorded in this prototype; that does not imply no real connections exist.

Search and category filters affect nodes and visible edges together. Following a hidden connection clears filters. Nodes have distinct access/community descriptions and primary-source links. Theme toggle and compact mobile list remain functional.

## Sources
- https://boardgamenightwg.com/faq/
- https://boardgamenightwg.com/community/
- https://bostonrobothackers.com/about.html
- https://bostonrobothackers.com/meetings.html
- https://www.massrobotics.org/
- https://www.massrobotics.org/events/
- https://www.massrobotics.org/women-in-robotics/
- https://fabhubkendall.fabfoundation.org/
- https://www.bu.edu/rastic/
- https://www.bu.edu/eng/academics/teaching-and-innovation/rastic/what-we-offer/

## Open questions
- Are the lines informative enough, or mostly describing calendar curation?
- Is a university community like RASTIC a useful node at this level?
- Should umbrella organizations and their named communities be separate nodes?

No verdict yet. Rewrite or discard this throwaway after the relationship model is settled.
