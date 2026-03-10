---
name: past-event
description: Add a past event entry to the past events page
user-invocable: true
---

# Past Event Skill

Add an entry to `content/pastevents.md` for an event that has already happened.

## Before Writing

Ask the user the following questions to gather details:

1. **Which chapter?** (Boston / Bay Area)
2. **What was the event date?** (e.g. March 10, 2026)
3. **Venue name?** (e.g. "Code Metal" or "Analog Garage @ Analog Devices")
4. **Venue address?** (street address, city, state, zip)
5. **Instagram post embed code?** (optional — the user can paste the full `<blockquote>...</blockquote>` embed HTML, or say "none" / "not yet")

## Entry Format

Each past event entry follows this exact format in `content/pastevents.md`:

### With Instagram embed:

```
# MM/DD/YYYY - Chapter
Venue Name \
Address Line 1, \
City, State Zip \
<blockquote class="instagram-media" ...>...</blockquote>
<script async src="//www.instagram.com/embed.js"></script>
```

### Without Instagram embed:

```
# MM/DD/YYYY - Chapter
Venue Name \
Address Line 1, \
City, State Zip
```

## How to Add the Entry

1. Read `content/pastevents.md` (just the first few lines — it's a large file) to confirm the frontmatter.
2. Insert the new entry **at the top of the file**, immediately after the frontmatter (`+++` block), with a blank line before and after.
3. Events are ordered newest-first (reverse chronological).
4. Date format in the heading is `MM/DD/YYYY` (e.g. `03/10/2026`).
5. Chapter name is exactly `Boston` or `Bay Area`.
6. Address lines are separated with ` \` (space-backslash) for line continuation. The last line before an Instagram embed also gets ` \`. The last line with no embed does NOT get a trailing ` \`.

## Notes

- The Instagram embed HTML can be very long — that's normal. Paste it exactly as provided by the user.
- If the user doesn't have the Instagram embed yet, add the entry without it. They can add it later.
- The `<script async src="//www.instagram.com/embed.js"></script>` line goes on its own line immediately after the closing `</blockquote>` tag.
- Only one `<script>` tag per entry (it's included with each embed block in the file).
- Look at existing entries in `content/pastevents.md` for reference if anything is unclear.
