# Board-game host coverage and source checks

Verified 2026-09-19.

## Coverage

The host audit covers the current Past Events and chapter pages, the public Luma calendar (all available past/future pages), and the full repository history of chapter announcements and About. External community events are excluded. Host aliases are deduplicated; a venue/lab and its parent organization are one entry.

There are 21 distinct hosts in dated club event records, plus Willow Garage as the historical origin host attested by [club history](https://boardgamenightwg.com/about/). Host history is a candidate audit, not an automatic inclusion rule: Griz clarified that Robodex is for robotics companies only. Non-company venues and community organizations below are deliberately excluded. Willow Garage remains an explicitly approved historical Easter egg, not a current hiring employer.

| Recorded host | Directory ID / scope decision | Event evidence |
| --- | --- | --- |
| Aeronaut Brewing Company | Excluded: brewery | [2023-11-16](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L202) |
| Analog Garage @ Analog Devices | `analog-devices` | [2026-05-21](https://luma.com/q7fpx3ol) |
| Apex.AI | `apex-ai` | [2024-03-28](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L176) |
| Artisans Asylum | Excluded: general makerspace | [2024-04-26](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L172) |
| Bear Robotics | `bear-robotics` | [2025-08-29](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L90) |
| Bonsai Robotics | `bonsai-robotics` | [2024-09-25](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L146) |
| Boston University RASTIC | Excluded: university center | [2026-09-09](https://luma.com/4mcqgsru) |
| Code Metal | `code-metal` | [2026-02-26](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L61) |
| Dusty Robotics | `dusty-robotics` | [2026-09-03](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L5) |
| Gaia AI | `gaia-ai` | [2024-05-24](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L167) |
| Hello Robot Inc | `hello-robot` | [2026-08-20](https://luma.com/2m8zbc0y) |
| InOrbit Robot Space | `inorbit` | [2024-06-05](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L162) |
| Intrinsic AI | `intrinsic` | [2026-01-14](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/5ecaeaef2e1a09f417ca2256a15c1452f5863b89/content/bayarea.md) |
| Locus Robotics | `locus-robotics` | [2025-11-06](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L76) |
| MassRobotics | Excluded: industry hub, not a robotics company | [2025-05-30](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L123) |
| Pickle Robot Company | `pickle-robot-company` | [2026-06-23](https://luma.com/1etgvzvp) |
| Polymath Robotics | `polymath-robotics` | [2026-04-16](https://luma.com/5ovtdgmq) |
| Reframe Systems | `reframe-systems` | [2026-07-22](https://luma.com/842o0xj3) |
| Robotics and AI Institute | `robotics-and-ai-institute` | [2026-03-26](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L54) |
| Silicon Valley Robotics | Excluded: industry association, not a robotics company | [2025-06-04](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L118) |
| Tutor Intelligence | `tutor-intelligence` | [2026-04-30](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/pastevents.md#L40) |
| Willow Garage | `willow-garage` | [2012–2014 origin history](https://github.com/boardgamenightwg/boardgamenightwg.github.io/blob/77f38b9f6831e3b990a56b915bc691193bc08a0b/content/about.md#L5) |

## Interpretation and limitations
- Cities are approximate municipality points, not office entrances. Charlestown is represented by Boston; Analog Garage uses Boston, not ADI’s Wilmington headquarters.
- Gaia AI has a company website and a MassRobotics resident listing, but no adequately verified current city source; no map pin is inferred from the facility footer. Historical Willow Garage is not assigned a current office pin.
- `careers_url: null` means no verified public employer careers page is listed, not proof that a company is not hiring. Permanent careers pages remain useful when no vacancies are posted. Analog Garage’s jobs link is explicitly ADI-wide.
- Polymath’s company-maintained YC hiring profile is corroborated by its official site, founder identities and accelerator membership; the main site currently has no careers link. Location evidence is its first-person employer job description, not a generic aggregator.
- Analog Devices sources were verified through real browser-rendered pages after direct-request timeouts. Other included official source requests were independently rechecked.

## Official source evidence

### Analog Devices (Analog Garage)
- Website: https://www.analog.com/en/who-we-are.html
- Careers: https://www.analog.com/en/careers.html
- Careers evidence: The live Garage page explicitly links to the employer's ADI careers page. Browser-rendered careers content identifies Analog Devices and links 'Search for Jobs' to https://analogdevices.wd1.myworkdayjobs.com/External. The verified employer careers landing page is used; the Workday HTML-only fetch was a blank JavaScript shell, not verified job listings. At Analog Devices, developing our people is as important as serving our customers. Together, we stay ahead of what’s possible.
- City source: https://www.analog.com/en/incubators/analog-garage.html
- Location quote: “Analog Garage
125 Summer Street, Suite 2100, Boston, MA 02110”
- Notes: Analog Garage is an Analog Devices innovation lab, not a separate employer. One employer entry should cover the host alias. The current official Garage page says 'Located in Downtown Boston at the Intersection of History and Innovation' and gives 125 Summer Street, Suite 2100. This local host location must not be replaced by the corporate HQ. Corporate HQ separately verified at https://investor.analog.com/contact-us: 'Corporate Headquarters Analog Devices, Inc. One Analog Way Wilmington, Massachusetts 01887' (whitespace normalized). Direct requests to www.analog.com timed out. Browser recovery successfully fetched actual About Us, Analog Garage, and Careers content. The root homepage redirected to /en/index.html and returned Access Denied; website therefore uses the verified official About Us page, not an unverified homepage body. The old /en/about-adi/careers.html URL redirects to /en/careers.html. Guessed /en/who-we-are/careers.html and old Garage URL variants returned real 404 pages and were rejected.

### Apex.AI
- Website: https://www.apex.ai/
- Careers: https://www.apex.ai/careers
- Careers evidence: Official careers page fetched successfully, describes the hiring process, and links 'View all open positions and apply from here' to https://jobs.lever.co/apex-ai? . It explicitly says: 'Apex.AI is a global company headquartered in Palo Alto, CA, developing breakthrough safe, certified, developer-friendly, and scalable software for mobility systems.'
- City source: https://www.apex.ai/legal-notice
- Location quote: “Apex.AI, Inc. 1881 Page Mill Road, Suite 103 Palo Alto, CA 94304, USA”
- Notes: Palo Alto is independently supported by current official About, Careers and Legal Notice pages. https://www.apex.ai/about says the company is based in Palo Alto, California, with other international offices. Use /careers, the verified main-nav careers landing page; the footer also links a different /openpositions URL, which was not needed or independently verified. This research verifies public company location, not whether the legal-notice street address is the past board-game event venue.

### Bear Robotics
- Website: https://www.bearrobotics.ai/
- Careers: https://www.bearrobotics.ai/careers
- Careers evidence: Official homepage links to /careers. Fetched careers page (HTTP 200) says 'Explore careers at Bear Robotics' and links 'View openings' to https://bear-robotics.breezy.hr. Fetched that branded employer board (HTTP 200), which lists Redwood City roles including 'Robotics Software Engineer II, Mission', 'Senior Robotics Software Engineer, Autonomy', and 'Technical Program Manager'.
- City source: https://www.bearrobotics.ai/privacy-policy
- Location quote: “please email us at privacy@bearrobotics.ai or write to us at 785 Broadway St, Redwood City, CA 94063, USA.”
- Notes: Official privacy policy is marked 'Last updated August 7, 2026'. Its public contact address and current company-linked job board independently support Redwood City. Use only a city-level pin. Summary quote from official homepage: 'Autonomous mobile robots for hospitality & logistics'. Breezy's raw HTML contains untranslated UI tokens, but employer identity, roles, and city labels were readable.

### Bonsai Robotics
- Website: https://www.bonsairobotics.ai/
- Careers: https://www.bonsairobotics.ai/careers/
- Careers evidence: Official careers page fetched successfully and says 'Apply today. Open roles'. It directly embeds the employer-controlled Rippling board at https://ats.rippling.com/embed/bonsairoboticsmain/jobs?s=https%3A%2F%2Fbonsairobotics.ai%2Fcareers%2F . The board and its Senior Machine Learning Engineer - Perception posting were fetched successfully and list San Jose, CA; other visible roles also list San Jose.
- City source: https://ats.rippling.com/en-US/bonsairoboticsmain/jobs/17740e0e-fb8c-48a6-a0fa-7bb2300b65ce
- Location quote: “The pay range for this role is: 150,000 - 220,000 USD per year (San Jose, CA) Apply now Software Engineering San Jose, CA”
- Notes: Current official branding remains Bonsai Robotics Inc.; it ACQUIRED farm-ng, rather than being acquired by farm-ng. Official July 24, 2025 release https://www.bonsairobotics.ai/news/bonsai-robotics-acquires-farm-ng-to-lead-the-future-of-autonomous-farming states: 'Existing shareholders of both companies will retain ownership stakes in the newly combined entity, Bonsai Robotics.' That release has a San Jose dateline and describes historical farm-ng as based in Watsonville. Prefer the current Bonsai employer careers page and current San Jose jobs over legacy farm-ng branding/locations. San Jose is verified as a current employer job location, not as a verified board-game venue street address; current main Contact page does not publish a street address. Summary supported by https://www.bonsairobotics.ai/about/ .

### Code Metal
- Website: https://www.codemetal.ai/
- Careers: https://www.codemetal.ai/careers
- Careers evidence: Official homepage links Careers to https://www.codemetal.ai/careers. The live page lists employer-specific engineering, research, operations, and other roles with application links on jobs.ashbyhq.com/code-metal. Boston appears on multiple roles, including the Boston-only Senior Engineering Manager (Modeling & Simulation) listing.
- City source: https://www.codemetal.ai/careers
- Location quote: “Senior Engineering Manager (Modeling & Simulation) Boston Apply”
- Notes: Checked 2026-09-19. City supported by current employer recruitment page; no claim about a specific street address or exclusive headquarters. Current careers page also lists San Francisco, remote and other locations. Identity and summary verified at https://www.codemetal.ai/ and https://www.codemetal.ai/about. MassRobotics also lists Code Metal in its resident directory.

### Dusty Robotics
- Website: https://www.dustyrobotics.com/
- Careers: https://www.dustyrobotics.com/careers
- Careers evidence: Official homepage links to /careers. Fetched careers page (HTTP 200) says: 'If you are interested in solving hard problems and working with awesome people in an impactful way, explore our current openings and take the first step towards joining our team.' It displays 'Create Your Own Job'.
- City source: https://www.dustyrobotics.com/privacy
- Location quote: “Dusty Robotics, Inc. 909 San Rafael Avenue Mountain View, CA 94043”
- Notes: Current public contact address comes from the official privacy policy, marked 'Last modified: 03/27/2026'; use a city-level pin, not the historical event address. Important: the careers page's linked Greenhouse posting https://boards.greenhouse.io/dustyrobotics/jobs/4698559003?gh_jid=4698559003 returned HTTP 404 and 'The job board you were viewing is no longer active.' Keep the verified permanent careers page, but do not assert active hiring. Summary evidence: https://www.dustyrobotics.com/about says 'Our flagship product, the FieldPrinter, automates layout'.

### Gaia AI
- Website: https://www.gaia-ai.eco/
- Careers: Not listed (null)
- Careers evidence: No official employer jobs page found in the live gaia-ai.eco homepage navigation or footer. https://www.gaia-ai.eco/careers returned a genuine HTTP 404 Page Not Found. The website links to LinkedIn company 79833755, but direct retrieval returned HTTP 999 and the browser showed an authentication wall. No ecosystem job board substituted.
- Current map location: omitted; no verified current city pin.
- Notes: Checked 2026-09-19. Exact forestry-company identity verified: MassRobotics' live resident directory has a 'Gaia AI' entry linking to https://www.gaia-ai.eco/ and describing its backpack-based LiDAR/computer-vision forest data platform, consistent with the live company homepage. IMPORTANT: the location quote is the MassRobotics facility footer, not Gaia AI's own published address. Boston is therefore provisional resident-directory evidence only; Gaia's homepage, privacy policy and terms do not establish a current city. Do not publish an unqualified current address based on this record. No acquisition, closure, or status change could be independently verified; absence of such an announcement on the fetched homepage is not proof of continued independent operation. Search attempts were blocked or unhelpful. www.gaiaai.com is a lander redirect and was rejected as the company website. This record is not the unrelated Gaia AI/crypto project.

### Hello Robot Inc
- Website: https://hello-robot.com/
- Careers: https://hello-robot.com/careers/
- Careers evidence: Official homepage links to /careers/. Fetched careers page (HTTP 200), headed 'Join Us!', says 'Contact us at info@hello-robot.com to hear about opportunities. Check back June 2026 for open roles!'
- City source: https://hello-robot.com/careers/
- Location quote: “We design, assemble, and test our robots in-house at our headquarters Martinez, CA (30-minute drive from Berkeley).”
- Notes: Use Martinez, not Berkeley: the live official careers page explicitly identifies its headquarters. It also mentions offices in Pittsburgh and Atlanta. The careers page is a valid permanent destination but has a stale June 2026 check-back message and no named openings in the retrieved content. Homepage identifies the company as 'Hello Robot Inc.' and describes Stretch 4 for assistive, research, and enterprise applications.

### InOrbit
- Website: https://www.inorbit.ai/
- Careers: https://www.inorbit.ai/company
- Careers evidence: Official company page has a Careers section: 'Join us in shaping the future of robot operations and orchestration.' It links employer postings for AI Researcher, Forward-Deployed Engineer and Account Executive. The AI Researcher and Forward-Deployed Engineer pages were fetched successfully at https://www.inorbit.ai/apply-ai-researcher and https://www.inorbit.ai/apply-forward-deployed-engineer . Use the company page as the verified careers landing page rather than inventing /careers or an unverified section anchor.
- City source: https://www.inorbit.ai/robotspace
- Location quote: “285 Castro St, Mountain View, California”
- Notes: Host venue is InOrbit Robot Space, not a separate employer; official venue page explicitly describes its community events and Mountain View location. Corporate HQ independently verified on official https://www.inorbit.ai/press/inorbit-business-execution-system : 'Headquartered in Mountain View, California, InOrbit.AI is a leading innovator in robot operations solutions.' The same headquarters sentence appears at https://www.inorbit.ai/press/inorbit-space-intelligence-optimizes-industrial-workflows . These are 2025 announcement pages still published on the current site; the September 30, 2025 Series A release https://www.inorbit.ai/press/series-a-investment also has a Mountain View dateline. Current venue page is the primary host-location evidence; do not assume its street address is also the corporate headquarters address. A typo /robdotspace appears in the site footer; verified correct venue URL is /robotspace.

### Intrinsic
- Website: https://www.intrinsic.ai/
- Careers: https://www.intrinsic.ai/careers
- Careers evidence: Official homepage links 'Working at Intrinsic' to /careers and 'Open roles' to /careers#roles. Fetched permanent careers page (HTTP 200), titled 'Careers at Intrinsic | Intrinsic', contains company culture, benefits, locations, and an 'open roles' section.
- City source: https://www.intrinsic.ai/careers
- Location quote: “With labs and offices in Mountain View, Munich and Singapore, we are bringing together a global team of experts with different backgrounds, skillsets and perspectives.”
- Notes: Verified the robotics company at intrinsic.ai, not another business named Intrinsic. City source explicitly describes a Mountain View office/lab; it does not label it headquarters. Main homepage says 'The team at Intrinsic is building a platform for the next generation of intelligent automation' and 'Intrinsic Flowstate is an all-in-one developer environment for building production-grade automation solutions.' Raw careers HTML includes placeholder role text/Lorem Ipsum for its dynamic listings, so this research does not claim a current opening count.

### Pickle Robot Company
- Website: https://www.picklerobot.com/
- Careers: https://www.picklerobot.com/careers
- Careers evidence: Actual Pickle careers content fetched, describing its team, benefits, and headquarters, with 'See Open Jobs' links to https://jobs.lever.co/picklerobot. The verified company careers landing page is used rather than claiming the outbound job board was inspected. Our headquarters are in a beautiful building in the heart of Charlestown, MA.
- City source: https://www.picklerobot.com/contact
- Location quote: “Visit us Pickle Robot Company 465 Medford Street, Suite 102 Charlestown, MA 02129”
- Notes: Both current contact and careers pages independently identify Charlestown; do not retain Cambridge from old event records. Charlestown is the Boston neighborhood/postal location used by the company.

### Polymath Robotics
- Website: https://www.polymathrobotics.com/
- Careers: https://www.ycombinator.com/companies/polymath-robotics/jobs
- Careers evidence: Fetched the company-specific YC jobs board and an employer-written vacancy (both HTTP 200). Board title: 'Jobs at Polymath Robotics | Y Combinator'; links to http://www.polymathrobotics.com, identifies founders Ilia Baranov and Stefan Seltz-Axmacher, and lists four roles, including 'Applications Engineer - San Francisco' and 'Senior Verification & Validation (V&V) Engineer - San Francisco', with functioning application links exposed on the page. The inspected job is explicitly 'San Francisco, CA, US'.
- City source: https://www.ycombinator.com/companies/polymath-robotics/jobs/AyzuzKw-applications-engineer-san-francisco
- Location quote: “Hybrid Workplace: Enjoy the flexibility of a hybrid work model, with engineering based in our SF Mission office three days a week, giving a good balance of maker time and synchronization time.”
- Notes: The company-controlled main website's /careers and /jobs return HTTP 404; its current navigation and sitemap do not expose a careers page. The careers URL supplied is its company-specific hiring presence on YC, not a generic job search. No outbound jobs-board link was found on the current main site; official identity is corroborated by the YC page's company website/founders matching https://www.polymathrobotics.com/about and the company launch post https://www.polymathrobotics.com/blog/launch explicitly stating 'we’re in Y Combinator this summer.' Location is from the company's detailed first-person job description on YC, not from a physical address on its own domain; it supports a San Francisco office, not an HQ assertion. Summary quote from https://www.polymathrobotics.com/about: 'Polymath builds core autonomy software for off-highway robots.' Google search returned a redirect/JavaScript shell and Bing returned unrelated results; neither was used as evidence.

### Reframe Systems
- Website: https://www.reframe.systems/
- Careers: https://jobs.ashbyhq.com/reframesystems
- Careers evidence: The official company navigation links to this Ashby board. The board returned its company-specific title; its public posting API returned 42 jobs, including Plant Manager - Net Zero Modular Construction in Andover, MA. Our volumetric modular buildings are produced in our Andover, MA microfactory, with a growing pipeline across North America and a new microfactory on the way.
- City source: https://www.reframe.systems/contact-form
- Location quote: “Reframe Systems Inc 30 Lowell Junction Rd, Andover, MA 01810 USA”
- Notes: Current contact page explicitly labels this address as Reframe headquarters. Do not carry an old Brookline event location into the current employer location. The live Plant Manager posting describes a future move from Andover to a Billerica factory 'Over the next year'; the current official headquarters address and job locations still say Andover. Do not treat the planned Billerica move as already complete. Official homepage and contact content fetched directly; the JavaScript-backed Ashby board was supplemented with its public posting API.

### Robotics and AI Institute
- Website: https://rai-inst.com/
- Careers: https://rai-inst.com/careers/
- Careers evidence: The official careers page was fetched with genuine RAI recruiting text, Cambridge and Zurich office descriptions, and available positions with individual Lever apply links. Machine Learning Engineer (Robotics and AI Institute LLC): on-site Full Time Cambridge, MA Apply
- City source: https://rai-inst.com/about/
- Location quote: “Cambridge, Massachusetts Our main campus is located in the innovation hub of Kendall Square. 145 Broadway, Cambridge, MA 02142”
- Notes: Aliases to consolidate into this one entry: RAI Institute, The AI Institute, Robotics and AI Institute. The fetched https://theaiinstitute.com URL redirects to https://rai-inst.com/ and returns the same RAI Institute company content. The current About page gives the Cambridge main campus at 145 Broadway and also identifies a Zurich office. Cambridge is the verified Boston-region location.

### Tutor Intelligence
- Website: https://tutorintelligence.com/
- Careers: https://jobs.lever.co/tutorintelligence
- Careers evidence: The official homepage links to this Lever board. Its actual company-branded job list was fetched, along with the Research Engineer detail page identifying Tutor Intelligence and an on-site Watertown role. Research Engineer On-site — Full-time Watertown, MA
- City source: https://jobs.lever.co/tutorintelligence/f772d72b-0662-410e-b708-ce7ed3b7f163
- Location quote: “Research Engineer Watertown, MA R&D - Research / Full-time / On-site”
- Notes: Watertown is supported by numerous current on-site engineering, research, manufacturing, and operations listings, rather than inferred from an old event address. Some roles on the same live board use the broader location 'Boston'. Watertown is the more specific verified local work location; this evidence does not independently establish a street address or formally designate headquarters.

### Willow Garage (historical)
- Website: https://boardgamenightwg.com/about/
- Careers: Not listed (null)
- Careers evidence: Historical host only; no current employer careers page is asserted.
- Current map location: omitted; no verified current city pin.
- Notes: Directly checked current club content/about.md from fetched main: the group began at Willow Garage in 2012 and it shut its doors in 2014. The website field intentionally links that public club history, not a current corporate homepage. The former corporate domain failed TLS hostname validation and is not used as a verified safe company link. No safe current company site, job page, or office location is claimed. Do not revive old office addresses as current map pins.

## City geocode provenance

Nominatim/OSM administrative relations, queried at no more than one request per second and cached. OSM data © OpenStreetMap contributors, ODbL. These are representative city/town coordinates, not rooftop geocodes. Existing directory cities reuse their earlier cached coordinates.

| Municipality | OSM relation | Latitude | Longitude |
| --- | --- | --- | --- |
| Andover, Massachusetts | [1840196](https://www.openstreetmap.org/relation/1840196) | 42.6571700 | -71.1408776 |
| Boston, Massachusetts | [2315704](https://www.openstreetmap.org/relation/2315704) | 42.3588336 | -71.0578303 |
| Cambridge, Massachusetts | [1933745](https://www.openstreetmap.org/relation/1933745) | 42.3656347 | -71.1040018 |
| Martinez, California | [112189](https://www.openstreetmap.org/relation/112189) | 38.0138934 | -122.1338674 |
| Mountain View, California | [1544956](https://www.openstreetmap.org/relation/1544956) | 37.3893889 | -122.0832101 |
| Palo Alto, California | [1544955](https://www.openstreetmap.org/relation/1544955) | 37.4443293 | -122.1598465 |
| Redwood City, California | [112309](https://www.openstreetmap.org/relation/112309) | 37.4863239 | -122.2325230 |
| San Francisco, California | [111968](https://www.openstreetmap.org/relation/111968) | 37.7879363 | -122.4075201 |
| San Jose, California | [112143](https://www.openstreetmap.org/relation/112143) | 37.3361663 | -121.8905910 |
| Somerville, Massachusetts | [1933746](https://www.openstreetmap.org/relation/1933746) | 42.3875968 | -71.0994968 |
| Watertown, Massachusetts | [1933719](https://www.openstreetmap.org/relation/1933719) | 42.3652576 | -71.1843234 |
