---
name: community-event
description: Add an approved community-submitted event to the Community Events page
user-invocable: true
---

# Community Event Skill

Add an approved community-submitted event to `content/community.md`. These are external events (not BGNWG-hosted) that members of the community submit through the form on the page.

## Before Writing

Ask the user the following questions to gather details:

1. **Which chapter?** (Boston / Bay Area)
2. **Event title?** (e.g. "Robotics Talk at MIT")
3. **Category?** (Robotics / Board Games / Tech & Community)
4. **Date and time?** (e.g. "June 15, 2026 @ 6:00 PM")
5. **Venue + address?** (venue name, street, city, state)
6. **Host / organizer?** (person, club, or company running the event)
7. **Short description?** (1–2 sentences)
8. **RSVP / info link?** (URL to a Luma, Eventbrite, Meetup, or other landing page)

## Entry Format

Each entry is a `###` heading with a category badge, followed by When / Where / Host lines and a short description, then a link:

```
### Event Title <span class="badge badge-{category}">{emoji} {Category Label}</span>
**When:** {date and time} \
**Where:** {venue}, {address} \
**Host:** {host} \
{1–2 sentence description}

[More info / RSVP →]({url})
```

### Category badge mapping

| Category | Badge class | Emoji + label |
|---|---|---|
| Robotics | `badge-robotics` | `🤖 Robotics` |
| Board Games | `badge-boardgames` | `🎲 Board Games` |
| Tech & Community | `badge-tech` | `💻 Tech & Community` |

## How to Add the Entry

1. Read `content/community.md` to see the current state.
2. Locate the correct chapter section (`## 🫘🌆 Boston` or `## 🌉🌅 Bay Area`).
3. If the section still shows the empty-state line (`_No community events posted yet — ..._`), replace it with the new entry. Otherwise, insert the new entry above existing entries if it's sooner, or below if it's later — events are listed in chronological order (soonest first).
4. Keep a blank line between entries.
5. Do not touch the example `<!-- ... -->` comment block — leave it as a reference for future entries.

## Cleanup

When an event's date has passed, remove its entry from the page. If a chapter section has no upcoming events left, restore the empty-state line:

```
_No community events posted yet — be the first to submit yours below!_
```

## Notes

- Only post approved submissions. Submissions come in via the Tally form embedded on the page; review them before adding here.
- Keep descriptions short — 1–2 sentences max. Link out for full details.
- If a submitted event clearly doesn't fit the audience (robotics / board games / adjacent tech) or the chapters (Boston / Bay Area), decline it rather than posting.
