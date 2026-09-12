/* Progressive enhancement only: content/community.md remains the event source. */
(() => {
  "use strict";

  const main = document.querySelector("main");
  if (!main) return;

  function enhance() {
    function element(tag, className, text) {
      const node = document.createElement(tag);
      if (className) node.className = className;
      if (text) node.textContent = text;
      return node;
    }

    // Read only an explicit English month/day[/day]/year prefix. This is a
    // display key, never a timestamp: the complete organizer schedule stays text.
    function displayDate(schedule) {
      const months = ["January", "February", "March", "April", "May", "June",
        "July", "August", "September", "October", "November", "December"];
      const match = schedule.match(/^([A-Za-z]+) (\d{1,2})(?:\s*[–-]\s*(\d{1,2}))?, (\d{4})(?=$|[\s,@;.])/);
      if (!match) return null;
      const month = months.indexOf(match[1]);
      const day = Number(match[2]);
      const end = Number(match[3] || match[2]);
      const year = Number(match[4]);
      const leap = year % 4 === 0 && (year % 100 !== 0 || year % 400 === 0);
      const days = [31, leap ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
      if (month < 0 || year < 1 || day < 1 || end < day || end > days[month]) return null;
      return { month: match[1], day, label: `${match[1]} ${match[4]}`,
        order: year * 10000 + (month + 1) * 100 + day };
    }

    // Work off-DOM. Unsupported source structure leaves the full original readable.
    const draft = main.cloneNode(true);
    const regions = [
      { key: "boston", name: "Boston", emoji: "🫘🌆", suffix: "Boston" },
      { key: "bayarea", name: "Bay Area", emoji: "🌉🌅", suffix: "Bay Area" },
      { key: "major", name: "Major Events", emoji: "🤖🌎", suffix: "Major Robotics Events" },
    ];
    const chapters = [];
    const events = [];
    let chapter;
    let row;
    let submission;

    for (const node of [...draft.childNodes]) {
      if (node.nodeName === "H2") {
        row = null;
        if (node.id === "submit-your-event") {
          submission = node;
          break;
        }
        const region = regions.find((r) =>
          node.textContent.trim().endsWith(r.suffix),
        );
        if (!region || chapters.some((c) => c.key === region.key)) return;
        const section = element("section", "community-chapter");
        section.setAttribute("aria-labelledby", node.id);
        node.before(section);
        section.append(node);
        chapter = { ...region, section, events: [] };
        chapters.push(chapter);
      } else if (node.nodeName === "H3" && chapter) {
        // A summary must not contain nested interactive controls.
        if (
          node.querySelector(
            "a, button, input, select, textarea, details, [tabindex]",
          )
        )
          return;
        const details = element("details", "community-event");
        const summary = element("summary");
        summary.append(node);
        details.append(summary);
        chapter.section.append(details);
        row = {
          details,
          summary,
          chapter,
          type: node.querySelector(".badge")?.textContent.trim() || "Other",
        };
        chapter.events.push(row);
        events.push(row);
      } else if (chapter) {
        (row ? row.details : chapter.section).append(node);
      }
    }
    if (!chapters.length || !submission) return;

    for (const event of events) {
      const metadata = event.details.querySelector("p");
      const labels = [...(metadata?.querySelectorAll("strong") || [])].map(
        (n) => n.textContent.trim(),
      );
      if (
        !["When:", "Where:", "Host:"].every(
          (label, index) => labels[index] === label,
        )
      )
        return;
      const when = metadata.querySelector("strong");
      if (!when || when.textContent.trim() !== "When:") return;
      let date = "";
      let node = when.nextSibling;
      while (node && node.nodeName !== "BR") {
        // A missing separator must not turn the next metadata label into a date.
        if (node.nodeName === "STRONG" || node.querySelector?.("strong")) return;
        date += node.textContent;
        node = node.nextSibling;
      }
      if (!node || !date.trim()) return;
      event.search = event.details.textContent.toLocaleLowerCase();
      event.date = displayDate(date.trim());
      const title = event.summary.querySelector("h3");
      const copy = element("div", "community-copy");
      const eyebrow = element("div", "community-eyebrow");
      eyebrow.append(element("span", "community-region-label", event.chapter.name));
      const badge = title.querySelector(".badge");
      if (badge) {
        // Preserve the original h3 verbatim; only its display badge is cloned.
        const displayBadge = badge.cloneNode(true);
        displayBadge.removeAttribute("id");
        displayBadge.querySelectorAll("[id]").forEach((node) => node.removeAttribute("id"));
        eyebrow.append(displayBadge);
      }
      copy.append(eyebrow, title, element("span", "community-date", date.trim()));
      if (event.date) {
        const tile = element("span", "community-date-tile");
        tile.setAttribute("aria-hidden", "true");
        tile.append(element("span", null, event.date.month.slice(0, 3)),
          element("b", null, String(event.date.day)));
        event.summary.append(tile);
      }
      event.summary.append(copy);
      const body = element("div", "community-detail");
      for (const node of [...event.details.childNodes]) {
        if (node !== event.summary) body.append(node);
      }
      event.details.append(body);
    }

    const layout = element("div", "community-layout");
    const sidebar = element("aside", "community-sidebar");
    const controls = element("div", "community-regions");
    controls.setAttribute("role", "group");
    controls.setAttribute("aria-label", "Event region");
    const helper = element("p", "community-helper", "Community-organized events.");
    helper.append(element("br"), "Club game nights stay on the ");
    const bostonLink = element("a", null, "Boston");
    bostonLink.href = "/boston/";
    const bayareaLink = element("a", null, "Bay Area");
    bayareaLink.href = "/bayarea/";
    helper.append(bostonLink, " and ", bayareaLink, " chapter pages.");
    sidebar.append(controls, helper);
    const agenda = element("div", "community-agenda");
    const chapterHead = element("div", "community-chapter-head");
    const chapterTitle = element("h2");
    const chapterDescription = element("p");
    chapterHead.append(chapterTitle, chapterDescription);
    const filters = element("div", "community-filters");
    const searchLabel = element("label");
    searchLabel.append(element("span", "community-sr-only", "Search events"));
    const search = element("input");
    search.type = "search";
    search.placeholder = "Search events, hosts, or venues…";
    searchLabel.append(search);
    const typeLabel = element("label");
    typeLabel.append(element("span", "community-sr-only", "Event type"));
    const type = element("select");
    const allTypes = element("option", null, "All types");
    allTypes.value = "";
    type.append(allTypes);
    for (const category of new Set(events.map((e) => e.type))) {
      const option = element("option", null, category);
      option.value = category;
      type.append(option);
    }
    typeLabel.append(type);
    filters.append(searchLabel, typeLabel);
    const status = element("p", "community-status");
    status.setAttribute("role", "status");
    status.setAttribute("aria-live", "polite");
    status.setAttribute("aria-atomic", "true");
    const empty = element(
      "p",
      "community-empty",
      "No events match these filters.",
    );
    const reset = element("button", "community-reset", "Reset filters");
    reset.type = "button";
    agenda.append(chapterHead, filters, status, empty, reset);
    layout.append(sidebar, agenda);
    chapters[0].section.before(layout);
    chapters.forEach((c) => agenda.append(c.section));
    const listing = element("div", "community-listing");
    const groups = new Map();
    // Merge months across regions for All regions, without cloning event nodes.
    const ordered = [...events].sort((a, b) =>
      (a.date?.order ?? Infinity) - (b.date?.order ?? Infinity));
    for (const event of ordered) {
      const label = event.date?.label || "Other dates";
      if (!groups.has(label)) {
        const section = element("section", "community-month");
        section.append(element("h2", null, label));
        groups.set(label, { section, events: [] });
        listing.append(section);
      }
      const group = groups.get(label);
      group.events.push(event);
      group.section.append(event.details);
    }
    agenda.append(listing);

    let selected = chapters.some((c) => c.key === "boston") ? "boston" : "all";
    const buttons = new Map();
    for (const region of [{ key: "all", name: "All regions" }, ...chapters]) {
      const button = element("button", null, region.name);
      button.type = "button";
      if (region.emoji) {
        const emoji = element("span", null, region.emoji);
        emoji.setAttribute("aria-hidden", "true");
        button.prepend(emoji, " ");
      }
      button.addEventListener("click", () => {
        selected = region.key;
        render();
      });
      buttons.set(region.key, button);
      controls.append(button);
    }

    function render() {
      const query = search.value.trim().toLocaleLowerCase();
      let count = 0;
      for (const event of events) {
        const matches =
          (selected === "all" || event.chapter.key === selected) &&
          (!type.value || type.value === event.type) &&
          event.search.includes(query);
        event.details.hidden = !matches;
        if (matches) count += 1;
      }
      for (const c of chapters) {
        c.section.hidden = selected !== "all" && selected !== c.key;
      }
      for (const group of groups.values()) {
        group.section.hidden = group.events.every((event) => event.details.hidden);
      }
      for (const [key, button] of buttons) {
        button.setAttribute("aria-pressed", String(key === selected));
      }
      const name =
        selected === "all"
          ? "All regions"
          : chapters.find((c) => c.key === selected).name;
      chapterTitle.textContent = name;
      chapterDescription.textContent = selected === "all"
        ? "Explore the whole community."
        : selected === "major" ? "Robotics gatherings worth the trip."
          : `Talks, demos, and game nights around ${name}.`;
      status.textContent = `${count} ${count === 1 ? "event" : "events"} · dates and times as listed by organizers`;
      status.append(element("span", "community-sr-only", ` · ${name}`));
      empty.hidden = count !== 0;
      reset.hidden = !query && !type.value;
    }

    search.addEventListener("input", render);
    type.addEventListener("change", render);
    reset.addEventListener("click", () => {
      search.value = "";
      type.value = "";
      render();
    });

    function revealFragment() {
      let id;
      try {
        id = decodeURIComponent(window.location.hash.slice(1));
      } catch {
        return;
      }
      // getElementById avoids interpreting fragment text as a CSS selector.
      const target = document.getElementById(id);
      if (!target || !main.contains(target)) return;
      const event = events.find((item) => item.details.contains(target));
      const c = event?.chapter || chapters.find((item) => item.section.contains(target));
      if (c) {
        selected = c.key;
        search.value = "";
        type.value = "";
        render();
        const details = target.closest("details");
        if (details) details.open = true;
      }
      target.scrollIntoView();
    }

    render();
    main.replaceChildren(...draft.childNodes);
    main.classList.add("community-page");
    window.addEventListener("hashchange", revealFragment);
    document.addEventListener("click", (event) => {
      if (
        event.defaultPrevented ||
        event.button !== 0 ||
        event.metaKey ||
        event.ctrlKey ||
        event.shiftKey ||
        event.altKey
      )
        return;
      const link = event.target.closest?.("a[href]");
      if (
        !link ||
        (link.target && link.target !== "_self") ||
        link.hasAttribute("download")
      )
        return;
      // Repeating the current fragment does not emit hashchange. Keep native
      // navigation (and the organizer's original href) untouched.
      if (
        link.origin === window.location.origin &&
        link.pathname === window.location.pathname &&
        link.search === window.location.search &&
        link.hash &&
        link.hash === window.location.hash
      )
        revealFragment();
    });
    revealFragment();
  }

  const original = [...main.childNodes];
  try {
    enhance();
  } catch {
    // This is optional UI, not a reason to lose access to an event or submission.
    main.replaceChildren(...original);
    main.classList.remove("community-page");
  }
})();
