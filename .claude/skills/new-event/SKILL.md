---
name: new-event
description: Draft a PR to add a new board game night event to the website
user-invocable: true
---

# New Event Skill

Draft a pull request to add a new board game night event to the website. Uses interactive menus to gather details — not all info is needed up front.

## Gather Details

Use the AskUserQuestion tool to collect info. Combine related questions into a single menu when possible (up to 4 questions per menu). The user may not have all details yet — that's OK. Accept partial answers and use "TBD" placeholders for anything unknown. Let the user type answers into the notes field rather than forcing them to pick from preset options — use options for genuinely distinct choices (like chapter selection), not as a gate before asking for free-text input.

### Menu 1: Chapter & Venue

Ask together:

- **Which chapter?** Options: Boston, Bay Area
- **Venue name, address, and website URL?** Let the user type what they know into notes. Options: "I have details" (type in notes), "TBD"

### Menu 2: Date, Time & RSVP

Ask together:

- **Event date and time?** Let the user type it (e.g. "April 23, 2026 @ 6:30 pm" or just "April"). Options: "I have a date" (type in notes), "TBD"
- **RSVP link?** Let the user paste the URL in notes. Options: "I have a link" (type in notes), "No link yet"

### Menu 3: Event Details & Logo

Ask together:

- **Any additional details?** (food/drink, alcohol policy, access instructions, photography policy, other notes for attendees). Let the user type what they know. Options: "I have some details" (type in notes), "All TBD for now"
- **Venue logo?** Options: "Yes — I'll provide file paths" (type in notes), "No logo yet", "Already in repo" (specify in notes)

If the user says logo is already in the repo, search `static/images/logos/` for matching files.

## After Gathering Details

1. Read the current chapter page (`content/boston.md` or `content/bayarea.md`) to understand the existing format.
2. Update the chapter page with the new event details, matching the style and structure of the existing content. Key patterns:
   - The `## Where` section has the venue name (linked to their website if available), address in bold, and optionally a Google Maps iframe embed.
   - If a venue logo is provided, include it with both light/dark mode variants using the `logo-light`/`logo-dark` CSS classes, wrapped in a link to the venue website.
   - The `## When` section has the date and time.
   - The `## RSVP` section has the RSVP link.
   - Additional sections (like `## Venue Info`, `## Access`) vary by chapter and event.
3. For any details not provided, use **visible** `TBD` placeholders on the page (not HTML comments). The page is often shared with the host for review, so TBD items serve as a checklist for them. Example:
   ```
   TBD: Food/drink info \
   TBD: Access/entry instructions \
   ```
4. If venue logo files are provided, place them in `static/images/logos/`.
5. If the user provides an address, look up the coordinates via web search and generate a proper Google Maps embed iframe with accurate coordinates and place ID.
6. Create a new git branch named like `add-<venue>-<month>-<year>-<chapter>` (e.g. `add-dusty-robotics-feb-2026-bayarea`).
7. Commit the changes with a message like: `Add <Venue> event for <Chapter> <Month> <Year>`.
8. Create a PR with:
   - Title: `Add <Venue> event for <Chapter> <Month> <Year>`
   - Body summarizing what was added and noting any TBD items that still need to be filled in.

## Notes

- Always read the existing chapter page before making changes so you match the current format.
- The two chapters (Boston and Bay Area) have slightly different styles — follow whichever one you're editing.
- Past events are tracked in `content/pastevents.md` — do NOT modify that file; it's updated after events happen.
- Commit messages in this repo follow the pattern: `Add <Venue> event for <Chapter> <Month> <Year>`.
- PR titles match the commit message pattern.
