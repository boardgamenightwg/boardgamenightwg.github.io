import {MONTHS, parseSchedule, clubState, selectEvents} from './homepage-model.mjs';

const allowedTags = new Set(['P', 'STRONG', 'B', 'EM', 'A', 'BR', 'SPAN']);

function element(tag, className = '', text = '') {
  const node = document.createElement(tag);
  node.className = className;
  node.textContent = text;
  return node;
}

// Project only the small prose vocabulary we display. Never clone source nodes,
// copy attributes, or assign source strings to innerHTML (even in a detached DOM).
function safeCopy(source, base) {
  if (source.nodeType === Node.TEXT_NODE) return document.createTextNode(source.textContent);
  if (source.nodeType !== Node.ELEMENT_NODE || !allowedTags.has(source.tagName)) {
    return document.createDocumentFragment();
  }
  const target = element(source.tagName.toLowerCase());
  if (source.tagName === 'A' && source.hasAttribute('href')) {
    try {
      const url = new URL(source.getAttribute('href'), base);
      if (['http:', 'https:'].includes(url.protocol)) target.href = url.href;
    } catch { /* Invalid links remain readable text. */ }
  }
  for (const child of source.childNodes) target.append(safeCopy(child, base));
  return target;
}

function proseText(node) {
  if (node.nodeType === Node.TEXT_NODE) return node.textContent;
  if (node.nodeName === 'BR') return '\n';
  return [...node.childNodes].map(proseText).join('');
}

function sectionNodes(heading) {
  const nodes = [];
  for (let node = heading.nextElementSibling; node && !/^H[23]$/.test(node.tagName); node = node.nextElementSibling) {
    nodes.push(node);
  }
  return nodes;
}

function chapterPreview(row, now) {
  const source = document.getElementById(`homepage-${row.dataset.chapter}-source`).content;
  const heading = [...source.querySelectorAll('h2')].find(h => h.textContent.trim() === 'When');
  const nodes = heading ? sectionNodes(heading) : [];
  if (nodes.some(n => n.tagName !== 'P' && n.textContent.trim())) return;
  const schedule = nodes.filter(n => n.tagName === 'P')
    .map(n => proseText(safeCopy(n, row.querySelector('a').href)).trim()).join('\n');
  if (!schedule && nodes.some(n => n.textContent.trim())) return;
  const state = clubState(schedule, row.dataset.region, now);
  const copy = row.querySelector('.club-schedule');
  if (state === 'soon') copy.textContent = 'Next date coming soon';
  if (state !== 'upcoming') return;
  copy.textContent = schedule;
  const where = [...source.querySelectorAll('h2')].find(h => h.textContent.trim() === 'Where');
  const venue = where && sectionNodes(where).flatMap(n => [...n.querySelectorAll('strong,b')])[0];
  if (venue) {
    copy.append(element('br'), document.createTextNode(proseText(safeCopy(venue, row.querySelector('a').href)).trim()));
  }
  row.querySelector('.chapter-link').textContent = 'Details / RSVP →';
}

function communityEvents(source, base) {
  let region = null;
  const events = [];
  for (const node of source.children) {
    if (node.tagName === 'H2') {
      const text = node.textContent.trim();
      region = text.endsWith('Boston') ? 'Boston' : text.endsWith('Bay Area') ? 'Bay Area' :
        text.endsWith('Major Robotics Events') ? 'Major Events' : null;
    }
    if (node.tagName !== 'H3' || !region) continue;
    const title = [...node.childNodes].filter(n => !(n.nodeType === Node.ELEMENT_NODE && n.classList.contains('badge')))
      .map(n => proseText(safeCopy(n, base))).join('').trim();
    const badge = node.querySelector('.badge');
    const category = ['robotics', 'boardgames', 'tech'].find(c => badge?.classList.contains(`badge-${c}`));
    const paragraphs = sectionNodes(node).filter(n => n.tagName === 'P').map(n => safeCopy(n, base));
    const text = paragraphs.map(proseText).join('\n');
    const schedule = text.match(/(?:^|\n)When:\s*([^\n]*)/)?.[1]?.trim() || '';
    events.push({title, region, category, badge: category ? proseText(safeCopy(badge, base)).trim() : '',
      schedule, dates: parseSchedule(schedule), paragraphs, id: node.id});
  }
  return events;
}

function eventRow(event, base) {
  const row = element('details', 'preview-event');
  const summary = element('summary');
  const tile = element('span', 'date-tile');
  tile.setAttribute('aria-hidden', 'true');
  const [, month, day] = event.dates.start.split('-');
  tile.append(element('span', '', MONTHS[Number(month) - 1].slice(0, 3)), element('b', '', String(Number(day))));
  const copy = element('span', 'event-copy');
  const eyebrow = element('span', 'eyebrow', event.region);
  if (event.category) eyebrow.append(element('span', `badge badge-${event.category}`, event.badge));
  copy.append(eyebrow, element('span', 'event-title', event.title), element('span', 'schedule', event.schedule));
  summary.append(tile, copy);
  const body = element('div', 'event-detail');
  body.append(...event.paragraphs);
  const link = element('a', '', 'View on our community page →');
  link.href = `${base}#${encodeURIComponent(event.id)}`;
  body.append(link);
  row.append(summary, body);
  return row;
}

// This async module sits after all source and destination markup. No fetch,
// DOMContentLoaded, theme, or inherited deferred-script dependency.
const now = new Date();
for (const row of document.querySelectorAll('.homepage .club-row')) chapterPreview(row, now);
const base = document.getElementById('homepage-directory-link').href;
const source = document.getElementById('homepage-community-source').content;
const events = selectEvents(communityEvents(source, base), now);
document.getElementById('homepage-events').replaceChildren(...events.map(e => eventRow(e, base)));
document.getElementById('homepage-directory-note').hidden = events.length > 0;
document.querySelector('.homepage').dataset.homepageReady = '';
