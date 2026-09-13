import test from 'node:test';
import assert from 'node:assert/strict';

let model;
try { model = await import('../static/scripts/homepage-model.mjs'); } catch {}

test('homepage exposes a date-only selection model', () => {
  assert.ok(model, 'Missing homepage event model');
});

test('strict calendar dates, ranges and secondary day schedules', () => {
  assert.ok(model, 'Missing homepage event model');
  for (const [text, start, end] of [
    ['September 12, 2026 @ 7 PM – midnight', '2026-09-12', '2026-09-12'],
    ['Sep 22–24, 2026, program hours vary (EDT)', '2026-09-22', '2026-09-24'],
    ['September 23, 2026 @ 7:30 AM – 6:30 PM; September 24 @ 7:30 AM – 4:00 PM', '2026-09-23', '2026-09-24'],
    ['October 20, 2026 @ 8 AM; October 21, 2026 @ 8 AM', '2026-10-20', '2026-10-21'],
    ['September 22–24, 2026, program hours vary (EDT). Opening-day registration: 7 AM; workshops: 8 AM', '2026-09-22', '2026-09-24'],
    ['February 29, 2028', '2028-02-29', '2028-02-29'],
  ]) {
    assert.deepEqual(model.parseSchedule(text), {start, end}, text);
  }
  for (const text of ['', 'Date TBD', '09/12/26', 'September 12, 20260', 'February 30, 2026', 'February 29, 2027', 'September 28–24, 2026', 'September 31, 2026', 'September 23, 2026; September 32 @ 7 AM', 'September 23, 2026; September 22 @ 7 AM', 'September 23, 2026; 09/24/26', 'September 23 to October 2, 2026', 'September 23, 2026; September 24, 20260', 'September 23, 2026; September 24, 2027']) {
    assert.equal(model.parseSchedule(text), null, text);
  }
});

test('calendar timezone not viewer timezone determines last-day eligibility', () => {
  assert.ok(model, 'Missing homepage event model');
  const now = new Date('2026-09-25T05:00:00Z');
  assert.equal(model.calendarToday('Boston', now), '2026-09-25');
  assert.equal(model.calendarToday('Major Events', now), '2026-09-25');
  assert.equal(model.calendarToday('Bay Area', now), '2026-09-24');
  const range = model.parseSchedule('September 22–24, 2026');
  assert.equal(model.isEligible(range, 'Bay Area', now), true);
  assert.equal(model.isEligible(range, 'Boston', now), false);
  assert.equal(model.isEligible(null, 'Boston', now), false);
  assert.equal(model.isEligible(model.parseSchedule('January 1, 2027'), 'Boston', now), true);
  assert.equal(model.calendarToday('Boston', new Date('2027-01-01T02:00:00Z')), '2026-12-31');
});

test('one nearest eligible per region then fill spare slots, sorted by start', () => {
  assert.ok(model, 'Missing homepage event model');
  const event = (id, region, schedule) => ({id, region, dates: model.parseSchedule(schedule)});
  const now = new Date('2026-09-24T18:00:00Z');
  const events = [
    event('old', 'Boston', 'September 1, 2026'),
    event('bay-later', 'Bay Area', 'September 28, 2026'),
    event('boston', 'Boston', 'September 26, 2026'),
    event('bay', 'Bay Area', 'September 23, 2026; September 24 @ 9 AM'),
    event('major', 'Major Events', 'September 22–24, 2026'),
    event('unknown', 'Boston', 'TBD'),
  ];
  assert.deepEqual(model.selectEvents(events, now).map(e => e.id), ['major', 'bay', 'boston']);
  assert.deepEqual(model.selectEvents(events.filter(e => e.region !== 'Boston'), now).map(e => e.id), ['major', 'bay', 'bay-later']);
  assert.deepEqual(model.selectEvents(events, new Date('2028-01-01T00:00:00Z')), []);
  assert.deepEqual(model.selectEvents([], now), []);
});

test('club state distinguishes missing, expired, unsupported and upcoming schedules', () => {
  assert.ok(model, 'Missing homepage event model');
  const now = new Date('2026-09-24T18:00:00Z');
  assert.equal(model.clubState('', 'Boston', now), 'soon');
  assert.equal(model.clubState('September 1, 2026', 'Boston', now), 'soon');
  assert.equal(model.clubState('TBD', 'Boston', now), 'unknown');
  assert.equal(model.clubState('September 22–24, 2026', 'Boston', now), 'upcoming');
});
