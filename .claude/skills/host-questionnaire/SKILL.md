---
name: host-questionnaire
description: Generate a questionnaire email to send to a potential host to collect event details
user-invocable: true
---

# Host Questionnaire Skill

Generate an email questionnaire to send to a potential board game night host to collect the details needed to set up an event page on the website.

## Gather Details

Use the AskUserQuestion tool to collect the following. Combine into a single menu:

- **Which chapter?** Options: Boston, Bay Area
- **Host/venue name?** Let the user type in notes. Options: "I have a name" (type in notes), "Not confirmed yet"
- **Host contact name?** (the person at the venue you're coordinating with) Let the user type in notes. Options: "I have a name" (type in notes), "Same as venue"
- **Tentative month/date?** (if one has been discussed) Let the user type in notes. Options: "I have a date in mind" (type in notes), "Not yet"

## Generate the Email

After gathering details, generate an email draft using the template below. Personalize the greeting with the host contact name if known. If a tentative date has been discussed, reference it in the email. If the chapter or venue is not yet confirmed, use generic placeholders.

## Template

```
Subject: Board Game Night at <venue> - Event Details Questionnaire

Hi <host contact>,

Thanks for volunteering to host Board Game Night! To get your event page set up on our website, I have a few questions. Don't worry if you don't have all the answers yet. Just let me know what you can and we'll fill in the rest as we go.

**Venue**
1. What is the full name of your company/venue?
2. What is the street address (including suite/floor if applicable)?
3. Do you have a website URL you'd like us to link to?

**Date and Time**
4. What date works for you? (We typically do weekday evenings.)
5. What start and end time would you prefer? (Most events run ~3 hours, e.g. 6:00 pm to 9:00 pm.)
6. Is there an RSVP deadline we should communicate to attendees?

**Food and Drink**
7. Will food or drinks be provided? If so, what are you planning? (Pizza and snacks are common.)
8. What is the alcohol policy? (Some venues are alcohol-free, others allow BYOB.)

**Capacity and Guest List**
9. Is there a maximum number of attendees?
10. Do you need a guest list or attendee names in advance? (Some venues require this for building security.)
11. Are there any visitor policies, security forms, or NDAs that attendees need to complete beforehand?
12. Is there a deadline for guest registration? (e.g. names must be submitted X days before the event.)

**Getting There**
13. How do attendees enter the building? (Buzzer codes, lobby pickup, specific entrances, etc.)
14. Are there any quirks with directions? (e.g. Google Maps sends you to the wrong side, use a specific door, etc.)
15. Any parking recommendations or public transit tips?
16. Who should attendees contact on the day of the event if they have trouble getting in? (Name and phone number or other contact method.)

**Other**
17. What is your photography/social media policy? (Can attendees take photos? Post on social media?)
18. Do you have a company logo you can share for the event page? (Ideally both a light and dark background version, PNG or SVG.)
19. Anything else attendees should know?

If it's easier, feel free to just reply inline with your answers. No need to be formal!

<coordinator>
Robotics Board Game Night - Working Group Coordinator

Google Group - https://groups.google.com/g/boardgamenightwg
Web Site - https://boardgamenightwg.com/
```

## Field Guide

- **venue**: Company or venue name. Use a placeholder like "[Venue]" if not yet confirmed.
- **host contact**: The person you're coordinating with. Use "there" if not known (i.e. "Hi there,").
- **coordinator**: The coordinator's name. Check memory or ask the user if not known.

## Notes

- This email collects the same information that the `new-event` skill needs to build an event page. The answers map directly to what goes on the chapter markdown page.
- Don't use dashes (emdashes, endashes, or hyphens) as punctuation in generated emails. Use commas, periods, or rephrase.
- Keep the tone friendly and low-pressure. Hosts are volunteering their space, so make it easy for them.
- If the user has already discussed some details with the host, offer to pre-fill those answers and skip those questions in the email.
