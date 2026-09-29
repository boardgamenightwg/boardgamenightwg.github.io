# Bay Area relationship audit

Research checked September 20, 2026. Scope: the existing BGNWG Bay Area,
Circuit Launch and HomeBrew Robotics Club profiles. This is evidence review for
[epic #96](https://github.com/boardgamenightwg/boardgamenightwg.github.io/issues/96),
not approval to change relationship types or expand the roster.

## Decision

**No new Bay Area edge is justified by this bounded audit under the existing
three types.** Keep the dataset unchanged; preserve the evidence and limitations
here so the next contributor need not repeat the same investigation. This is
not a claim that these organizations have no real-world relationships.

| Pair | Finding | Action |
| --- | --- | --- |
| BGNWG Bay Area / Circuit Launch | Event venue and historical co-location context; named organizer is Silicon Valley Robotics. | Do not infer sharing Circuit Launch's own event, joint organization or operation. |
| BGNWG Bay Area / HBRC | No qualifying public evidence found in the inspected club and HBRC sources. | Leave unconnected; unknown, not disproven. |
| Circuit Launch / HBRC | No qualifying public evidence found in the inspected official pages/blog and HBRC sources; Circuit Launch calendar retrieval was inconclusive. | Leave unconnected; unknown, not disproven. |

Add an edge only when a public primary source establishes the endpoints and the
particular relationship being asserted.

## Venue is not organizer

The club's Community Events page lists **Automated Happy Hour with Rodney Brooks**
on September 22, 2026 at Circuit Launch in Mountain View. Its host field names
**Silicon Valley Robotics / Robots and Startups, with Automated Podcast**, not
Circuit Launch.[1]

> Host: Silicon Valley Robotics / Robots and Startups, with Automated Podcast

This is evidence of the club sharing that event and of its advertised venue.
It does **not**, by itself, support `bgn-bay → circuit-launch` as
`shares_events_from`, nor joint organization or operation. That would confuse
the venue with the group whose event is shared. Do not draw an edge to Circuit
Launch merely to make the Bay Area graph look connected.

The organizer's dedicated event page independently thanks Circuit Launch as
**venue hosts**, while describing the evening with the Automated Podcast crew
and Silicon Valley Robotics.[4]

> Big thanks to Automated Podcast, A3, and to our venue hosts at Circuit Launch

### Historical naming correction matters

The club's preserved June 4, 2025 announcement names **Silicon Valley Robotics**
and says the board-game event would occur simultaneously with its **Bots & Beer**
event. The page still uses Circuit Launch's logo/link and mapped venue.[5]

> The event will occur simultaniously with the Silicon Valley Robotics "Bots & Beer" Event.

The spelling above is the source's. The corresponding correction is explicitly
named **Renamed to Silicon Valley Robotics where necessary (#25)**.[6]
The public Past Events record likewise names Silicon Valley Robotics for
June 4, 2025.[3]

This is dated co-location/event context. It is not evidence that Circuit Launch
operates BGNWG, or that the two jointly organized the June 4 game night. Preserve
the organizer/venue distinction instead of resurrecting an earlier attribution.

## HomeBrew Robotics Club meeting context

The HBRC homepage's September 30 meeting announcement identifies its in-person
session at **Maker Nexus, 1330 Orleans Drive, Sunnyvale** and describes its
meetings as hybrid.[2]

> This month’s in person session is at

> Maker Nexus, 1330 Orleans Drive, Sunnyvale, CA, 94089

This gives participation context, not an HBRC–Circuit Launch connection. It also
does not establish that HBRC and Circuit Launch have never worked together.
Maker Nexus is outside this audit's existing-node scope; this observation is not
an automatic proposal to add it or classify a new relationship.

## Coverage and limits

The bounded review inspected the club's chapter, Community Events, Past Events
and About pages; GitHub history for the Bay Area and Community pages; the public
club Luma calendar's returned past/future entries; Circuit Launch's official
FAQ/About pages and blog posts exposed by its sitemap; and HBRC's homepage,
About, Get Involved, membership, resources, challenges and linked meeting-archive
pages. Searches within the retrieved text looked for the other two organizations,
HBRC, HomeBrew variants and board-game references. This is not an exhaustive
review of every newsletter, external discussion thread or deleted event page.

Specific limitations:

- Circuit Launch's calendar loaded its controls but no event entries in the
  research browser, with JavaScript errors. This was an **inconclusive retrieval**,
  not evidence that its calendar has no relevant events.[8]
- HBRC's Get Involved page still says meetings are Zoom-only until the health
  emergency ends, while its current homepage announces hybrid meetings. Use
  current dated announcements for attendance, not that older blanket statement.[7][2]
- The live club Community page and HBRC homepage are rolling sources. The dated
  observations above describe what was retrieved on September 20, 2026; later
  disappearance of a listing is not evidence that it never existed. The club's
  2025 announcement is linked at its fixed Git revision.[1][2][5]
- No private mail, member lists, login-only content or personal relationships were
  used. Public event attendance/co-location does not establish organizational
  collaboration, operation or membership overlap.

## Model question for review

The useful distinction exposed by the event listing is **organizer versus venue**.
If venue relationships become a requirement, bring a separate proposal with
explicit direction, event/date scope and evidence requirements. Do not stretch
`runs_activities_with` to cover a venue listing or use `operates` as a synonym
for hosting. An isolated profile with honest participation information is better
than a misleading connection.

The three existing types remain provisional. This research does not settle
university/community inclusion, authorize Boston candidates, or convert Megamap
into the company/jobs directory maintained separately as Robodex.

## Sources

[1] https://boardgamenightwg.com/community
[2] https://www.hbrobotics.org
[3] https://boardgamenightwg.com/pastevents
[4] https://luma.com/n688dcl5
[5] https://raw.githubusercontent.com/boardgamenightwg/boardgamenightwg.github.io/bfcb7064b4b5b05585fcb1870f97f8529f3678c0/content/bayarea.md
[6] https://api.github.com/repos/boardgamenightwg/boardgamenightwg.github.io/commits/bfcb7064b4b5b05585fcb1870f97f8529f3678c0
[7] https://www.hbrobotics.org/get-involved
[8] https://circuitlaunch.com/calendar/upcoming-events-day.html
