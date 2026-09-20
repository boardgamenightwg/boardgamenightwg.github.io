# Robodex: multi-region and ROSCon 2026 source audit

Public sources checked 2026-09-19. Conference sponsorship is candidate discovery,
not evidence of a local office. This is a bounded audit, not an exhaustive list
of ROSCon participants or every company's worldwide offices.

## One company, multiple regional views

Schema version 2 keeps one canonical company ID, name, website, careers link,
summary and news list. `regions` controls regional membership; `locations` holds
independently sourced city pins keyed by region. A missing regional location
must not borrow another region's pin. All pins remain approximate city points.

### Generalist AI: Boston and Bay Area

The company states: “We are located in the Bay Area (CA) and Boston (MA).”[15]
Its employer-controlled Ashby feed gives **Boston (Somerville)** for on-site
Mechanical Engineer and Data Collection Lab Manager roles, with structured
`addressLocality` of `Somerville`.[18]
The Office Manager role gives **San Francisco Bay Area (San Mateo)**,
`workplaceType: OnSite`, and `addressLocality: San Mateo`.[18]

Use Somerville and San Mateo, not the earlier San Francisco pin inferred from
its less-specific “San Francisco (SFO)” website label. The shared careers link
and news remain stored once. Human-readable job sources are linked from each
regional location in the data; the employer feed above preserves the exact
municipality evidence.

### MathWorks: Boston and Bay Area

The official worldwide office directory identifies both “MathWorks Office
Natick, Apple Hill Campus” and “MathWorks Office Santa Clara 3975 Freedom Circle
Santa Clara, CA 95054”.[16]
Retain Natick and add Santa Clara under the same company record. Its existing
robotics-software description is unchanged.

## Additions

### Eka Robotics — Boston / Cambridge

The official website describes a Vision-Force-Action robot foundation model and
links directly to its Ashby careers board and `eka-robotics` LinkedIn company
page.[10]
The publicly accessible company LinkedIn page states “Headquarters Cambridge,
MA” and “Cambridge, MA 02139, US”.[12]
The official hiring feed independently gives `Boston Area`, `OnSite`, and
structured `addressLocality: Cambridge` for its roles.[20]

Include Boston only: no Bay office was verified. This is Eka Robotics at
`ekarobotics.com`, not a similarly named EKA company, Ekumen, or Eureka Robotics.
The LinkedIn check used the public company overview after dismissing its
optional sign-in prompt; no account login or employee-profile access was needed.

### Innate — Bay Area / Palo Alto

The ROSCon 2026 sponsor page links to Innate.[1]
Its official website describes MARS and an open robot operating system, links
to its company Ashby board, and says “Made with 💙 in Palo Alto”.[17]
Its employer feed corroborates `OnSite` roles with structured municipality
`Palo Alto`.[19]

### Simbe Robotics — Bay Area / Burlingame

The ROSCon 2026 sponsor page links to Simbe Robotics.[1]
Its official website describes Tally autonomous shelf-scanning robots, its
careers page links to the employer Lever board, and its contact page explicitly
names **Burlingame, CA**.[2][4][5]
The employer board independently lists Bay Area on-site robotics and IT roles.[8]

The contact page pairs Burlingame with postal code 94080; only its explicit city
name is used, not that inconsistent postal code or an inferred street address.
LinkedIn's Simbe page hit an authentication wall during verification, so it is
not used as location evidence. The official company contact page supplies the
city; the ordinary city geocode is used rather than a postal-code search.

### Ouster — Bay Area / San Francisco

The ROSCon 2026 sponsor page links to Ouster.[1]
Its official careers page describes sensors and tools for roboticists and links
to the employer job board.[6]
Its official investor site explicitly states “Headquartered in San Francisco,
CA” and provides a San Francisco company contact address.[21]
This headquarters evidence, not a job's location alone, supports the city pin.

### Foxglove — Bay Area, no physical-office pin

The ROSCon 2026 sponsor page links to Foxglove.[1]
Its official careers page describes robotics development tools, and its privacy
page identifies a company contact address in San Francisco.[22][23]
The public company LinkedIn page additionally identifies San Francisco as its
headquarters.[24] A physical office was not verified;
keep the regional row usable with **Location not mapped**, rather than turn a
possible mailing address into an office pin.

## Coverage and limits

- Reviewed the official ROSCon 2026 sponsor links. The additional regional
  companies selected in this pass are Innate, Simbe, Ouster and Foxglove.[1]
- The public ROSCon program was also inspected during research. No positive Eka
  conference affiliation was established; inclusion does not imply sponsorship,
  attendance, speaking, or a board-game-host relationship.
- Existing Intrinsic, Hello Robot and Locus Robotics also appear in the
  conference's company links; they are not duplicated as new companies.[1]
- No additional Boston membership was verified for Intrinsic. Symbotic and
  Analog Devices office checks were incomplete or blocked; their existing
  memberships remain unchanged. Missing evidence is not evidence of absence.
- LinkedIn company pages for Eka, Generalist and Foxglove were publicly readable
  during research. Some other pages returned HTTP 999 or sign-in walls. No
  authentication or challenge was bypassed.
- All careers destinations remain company-level pages, not individual vacancies.
  Individual jobs are used only as evidence where their on-site location is
  explicit. No private correspondence or personal contact details are published.

## Coordinate provenance

Coordinates use the cached OpenStreetMap/Nominatim municipality results, matched
by exact municipality name, US state, and city/town/municipality address type.
This avoids the separate San Diego neighborhood also called Burlingame. City
points are approximate and do not identify an office entrance.

| Municipality | Latitude | Longitude | OSM municipal feature |
| --- | --- | --- | --- |
| Somerville, Massachusetts | 42.3875968 | -71.0994968 | https://www.openstreetmap.org/relation/1933746 |
| San Mateo, California | 37.5629997 | -122.3253265 | https://www.openstreetmap.org/relation/2835017 |
| Santa Clara, California | 37.3541132 | -121.9551740 | https://www.openstreetmap.org/relation/2221647 |
| Cambridge, Massachusetts | 42.3656347 | -71.1040018 | https://www.openstreetmap.org/relation/1933745 |
| Palo Alto, California | 37.4443293 | -122.1598465 | https://www.openstreetmap.org/relation/1544955 |
| Burlingame, California | 37.5780965 | -122.3473099 | https://www.openstreetmap.org/relation/9949457 |
| San Francisco, California | 37.7879363 | -122.4075201 | https://www.openstreetmap.org/relation/111968 |

## Sources

[1] https://roscon.ros.org/2026
[2] https://www.simberobotics.com
[4] https://www.simberobotics.com/about/work-at-simbe
[5] https://www.simberobotics.com/about/contact-us
[6] https://ouster.com/company/careers
[8] https://jobs.lever.co/SimbeRobotics
[10] https://ekarobotics.com
[12] https://www.linkedin.com/company/eka-robotics
[15] https://generalistai.com/about
[16] https://www.mathworks.com/company/aboutus/contact_us.html
[17] https://www.innate.bot
[18] https://api.ashbyhq.com/posting-api/job-board/generalist
[19] https://api.ashbyhq.com/posting-api/job-board/innate
[20] https://api.ashbyhq.com/posting-api/job-board/ekarobotics
[21] https://investors.ouster.com
[22] https://foxglove.dev/careers
[23] https://foxglove.dev/legal/privacy
[24] https://www.linkedin.com/company/foxglovedev
