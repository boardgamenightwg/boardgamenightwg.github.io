// Calendar dates only: organizer clock times are display text, never timestamps.
export const MONTHS = ['January', 'February', 'March', 'April', 'May', 'June',
  'July', 'August', 'September', 'October', 'November', 'December'];
const monthPattern = MONTHS.map(m => `${m}|${m.slice(0, 3)}`).join('|');
const datePattern = new RegExp(`^(${monthPattern}) (\\d{1,2})(?:\\s*[–-]\\s*(\\d{1,2}))?(?:, (\\d{4}))?(?=$|\\s*@|,\\s*[A-Za-z])`, 'i');

function calendarDate(year, month, day) {
  const date = new Date(Date.UTC(year, month - 1, day));
  if (year < 1000 || date.getUTCFullYear() !== year ||
      date.getUTCMonth() + 1 !== month || date.getUTCDate() !== day) return null;
  return `${year}-${String(month).padStart(2, '0')}-${String(day).padStart(2, '0')}`;
}

export function parseSchedule(schedule) {
  const parts = schedule.trim().split(';');
  let start, end, year, month;
  for (const [index, part] of parts.entries()) {
    const text = part.trim();
    // Named schedule clauses (e.g. "workshops: 8 AM") are not extra dates.
    if (index > 0 && /^[a-z -]+:/i.test(text)) continue;
    const match = text.match(datePattern);
    if (!match || (index === 0 && !match[4])) return null;
    const nextYear = Number(match[4] || year);
    const nextMonth = MONTHS.findIndex(m => m.toLowerCase().startsWith(match[1].toLowerCase())) + 1;
    // Only same-month secondary dates are supported; no rollover inference.
    if (index > 0 && (nextYear !== year || nextMonth !== month)) return null;
    year = nextYear;
    month = nextMonth;
    const first = calendarDate(year, month, Number(match[2]));
    const last = calendarDate(year, month, Number(match[3] || match[2]));
    if (!first || !last || last < first || (end && first < end)) return null;
    start ||= first;
    end = last;
  }
  return start ? {start, end} : null;
}

export function calendarToday(region, now = new Date()) {
  const timeZone = region === 'Bay Area' ? 'America/Los_Angeles' : 'America/New_York';
  const parts = new Intl.DateTimeFormat('en-US', {
    timeZone, year: 'numeric', month: '2-digit', day: '2-digit',
  }).formatToParts(now);
  const values = Object.fromEntries(parts.map(p => [p.type, p.value]));
  return `${values.year}-${values.month}-${values.day}`;
}

export function isEligible(dates, region, now = new Date()) {
  return Boolean(dates && dates.end >= calendarToday(region, now));
}

export function clubState(schedule, region, now = new Date()) {
  if (!schedule.trim()) return 'soon';
  const dates = parseSchedule(schedule);
  if (!dates) return 'unknown';
  return isEligible(dates, region, now) ? 'upcoming' : 'soon';
}

export function selectEvents(events, now = new Date()) {
  const eligible = events.filter(e => isEligible(e.dates, e.region, now))
    .sort((a, b) => a.dates.start.localeCompare(b.dates.start));
  const selected = [];
  for (const region of ['Boston', 'Bay Area', 'Major Events']) {
    const nearest = eligible.find(e => e.region === region);
    if (nearest) selected.push(nearest);
  }
  for (const event of eligible) {
    if (selected.length === 3) break;
    if (!selected.includes(event)) selected.push(event);
  }
  return selected.sort((a, b) => a.dates.start.localeCompare(b.dates.start));
}
