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

    // Work off-DOM. Unsupported source structure leaves the full original readable.
    const draft = main.cloneNode(true);
    const regions = [
      { key: "boston", name: "Boston", suffix: "Boston" },
      { key: "bayarea", name: "Bay Area", suffix: "Bay Area" },
      { key: "major", name: "Major Events", suffix: "Major Robotics Events" },
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
      // Copy text, never interpret HTML, parse dates, infer a timezone, or truncate a range.
      event.summary.append(element("span", "community-date", date.trim()));
      event.search = event.details.textContent.toLocaleLowerCase();
    }

    const layout = element("div", "community-layout");
    const sidebar = element("aside", "community-sidebar");
    const controls = element("div", "community-regions");
    controls.setAttribute("role", "group");
    controls.setAttribute("aria-label", "Event region");
    sidebar.append(controls);
    const agenda = element("div", "community-agenda");
    const filters = element("div", "community-filters");
    const searchLabel = element("label", null, "Search events");
    const search = element("input");
    search.type = "search";
    search.placeholder = "Title, date, host, or venue…";
    searchLabel.append(search);
    const typeLabel = element("label", null, "Event type");
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
    agenda.append(filters, status, reset, empty);
    layout.append(sidebar, agenda);
    chapters[0].section.before(layout);
    chapters.forEach((c) => agenda.append(c.section));

    let selected = chapters.some((c) => c.key === "boston") ? "boston" : "all";
    const buttons = new Map();
    for (const region of [{ key: "all", name: "All regions" }, ...chapters]) {
      const button = element("button", null, region.name);
      button.type = "button";
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
      for (const [key, button] of buttons) {
        button.setAttribute("aria-pressed", String(key === selected));
      }
      const name =
        selected === "all"
          ? "All regions"
          : chapters.find((c) => c.key === selected).name;
      status.textContent = `${count} ${count === 1 ? "event" : "events"} · ${name} · dates and times as listed by organizers`;
      empty.hidden = count !== 0;
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
      const c = chapters.find((item) => item.section.contains(target));
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
