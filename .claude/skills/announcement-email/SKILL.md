---
name: announcement-email
description: Generate an announcement email for an upcoming board game night event
user-invocable: true
---

# Announcement Email Skill

Generate an announcement email for an upcoming board game night event. Read the relevant chapter page (e.g. `content/boston.md`, `content/bayarea.md`) to extract event details, then fill in the template below.

## Template

```
Board game night for the <chapter> chapter in <month> will be held at

<host with website> on <date> @ <time>

Additional details available on the boardgamenightwg website including the link for RSVP.

<event details>
RSVP conditions
Food/Alcohol information
Contact info if coordinators need to get attendees
</event details>

Anyone interested in hosting for <next month> is encouraged to contact me at their earliest convenience.

<coordinator>
Robotics Board Game Night - Working Group Coordinator

Google Group - https://groups.google.com/g/boardgamenightwg
Web Site - https://boardgamenightwg.com/
```

## Field Guide

- **chapter**: City name (e.g. "Boston", "Bay Area")
- **month**: Month of the event
- **host with website**: Venue name hyperlinked or with URL
- **date / time**: From the "When" section of the chapter page
- **event details**: Pull from the chapter page — RSVP requirements, food/drink info, photography rules, alcohol policy, how attendees will be contacted or checked in, etc.
- **next month**: The calendar month after the event
- **coordinator**: Ask the user if not known

## Notes

- The chapter page URL follows the pattern: `https://boardgamenightwg.com/<chapter>/`
- Always check the chapter markdown file for the latest event details before generating
- If coordinator name is not known, leave a placeholder and ask
