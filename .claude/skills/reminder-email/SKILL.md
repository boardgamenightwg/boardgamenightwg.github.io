---
name: reminder-email
description: Generate a reminder email for an upcoming board game night event
user-invocable: true
---

# Reminder Email Skill

Generate a reminder email for an upcoming board game night event. Read the relevant chapter page (e.g. `content/boston.md`, `content/bayarea.md`) to extract event details, then fill in the template below.

## Template

```
Subject: <chapter> Chapter Board Game Night <date> Reminder | <host/location>

Hey everyone,

Just a reminder that the <chapter> Chapter Board Game Night Working Group for <month> will be hosted by <host> on <date> @ <time>.

<flavor text - a sentence or two of color about the host, venue, or what to expect>

Please remember to RSVP at https://boardgamenightwg.com/<chapter slug>

Reminder, we're still looking to solicit locations at robotics companies, makerspaces, etc so if you have any suggestions, feel free to contact me.

<coordinator>
Robotics Board Game Night - Working Group Coordinator

Google Group - https://groups.google.com/g/boardgamenightwg
Web Site - https://boardgamenightwg.com/
```

## Field Guide

- **chapter**: City name (e.g. "Boston", "Bay Area")
- **chapter slug**: URL-safe chapter name (e.g. "boston", "bayarea")
- **month**: Month of the event
- **host**: Venue/company name
- **host/location**: Short identifier for subject line (e.g. "Tutor Intelligence", "Bear Robotics - Redwood City")
- **date / time**: From the "When" section of the chapter page
- **flavor text**: A brief, friendly note about the host company, the venue, or something to look forward to. Keep it casual and warm.
- **coordinator**: Ask the user if not known

## Notes

- The chapter page URL follows the pattern: `https://boardgamenightwg.com/<chapter slug>/`
- Always check the chapter markdown file for the latest event details before generating
- If coordinator name is not known, leave a placeholder and ask
- This is a reminder email, not an announcement - assume recipients already know about the event and just need the nudge with key details
- Don't use dashes (emdashes, endashes, or hyphens) as punctuation in generated emails. People don't write that way. Use commas, periods, or just rephrase.
