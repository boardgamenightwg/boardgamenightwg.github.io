---
name: event-lifecycle
description: How events flow through the site, Luma calendar, and post-event recap. Read before any cross-page event work
user-invocable: true
---

# Event Lifecycle Skill

Orientation guide for how an event moves through this repo, the club's Luma
calendar, and the post-event recap flow. The other skills in this folder are
per-page procedures; this one is the map that connects them. Read this before
doing any work that touches more than one page or that involves the Luma
calendar.

## The lifecycle

1. **New club game night.** Host questionnaire (see `host-questionnaire`) →
   chapter page update (see `new-event`) → Luma event created → announcement
   email (see `announcement-email`) → reminder email a few days before (see
   `reminder-email`). Chapter pages (`content/boston.md`,
   `content/bayarea.md`) show ONLY the next upcoming event.
2. **Community events.** Approved external events go on BOTH
   `content/community.md` AND the Luma calendar (see `community-event`).
   Entries are listed in chronological order within the
   `## 🫘🌆 Boston` / `## 🌉🌅 Bay Area` sections.
3. **After the date passes.** The community.md entry comes DOWN. The Luma
   side ages out of the calendar embed on its own — do not delete past events
   from Luma.
4. **Day after a club game night.** Instagram recap post (photos + caption,
   see `instagram-caption`) and a `content/pastevents.md` entry (see
   `past-event`), newest-first. The IG embed block is added to the past-events
   entry only after the photos are actually posted and the IG URL is known.

## Ground truth locations

- **Site:** Zola static site, deployed via GitHub Pages on push to `main`.
  Content lives in `content/`; page formats are defined by the skills in this
  folder.
- **Luma calendar (read-only, no API key needed):**
  `curl -s "https://api.lu.ma/calendar/get-items?calendar_api_id=cal-v6H3Jm84BrwuOYb&period=future"`
  → JSON with `.entries[].event` (`name`, `start_at`, `url`,
  `geo_address_info`). `period=past` also works. Writing events to the
  calendar is done manually in the Luma admin (see `community-event`).
- **Timezone:** all event-date math is America/New_York. Luma timestamps are
  UTC; convert before comparing dates.

## Sync rules (community.md ↔ Luma)

- A page entry linking `luma.com/<slug>` must match a Luma event with the
  same `event.url`. Non-Luma links (Eventbrite, Meetup, etc.) cannot be
  matched automatically; flag them for a manual Luma add.
- A page entry with no Luma counterpart → flag for manual add to the
  calendar.
- A Luma event with no community.md match → first check it isn't a
  club-hosted game night (those belong on a chapter page, e.g. "Board Game
  Night @ BU RASTIC" belongs in `content/boston.md`). Only genuinely
  unmatched external events are community-page candidates.
- A page entry whose date has passed → remove it. If a chapter section
  becomes empty, restore the empty-state line:
  `_No community events posted yet — be the first to submit yours below!_`

## PR conventions

- Never push directly to `main`; always open a PR.
- Branch names: `add-<venue>-<month>-<year>-<chapter>` for new events,
  `add-past-event-<venue>-<date>` for past-event entries,
  `remove-expired-community-events-<date>` for cleanup.
- Commit/PR title patterns: `Add <Venue> event for <Chapter> <Month> <Year>`
  for adds; `Remove expired community event(s) <date>` for cleanup.

## Pitfalls

- `content/pastevents.md` is very large (Instagram embed HTML, ~160KB). Read
  only the frontmatter, and insert new entries right after the `+++` block.
  Never dump the whole file.
- `content/community.md` has an HTML-commented example entry at the bottom.
  Never edit it and never treat it as a live event.
- The two chapters have slightly different page styles — always follow the
  style of the chapter page you are editing.
