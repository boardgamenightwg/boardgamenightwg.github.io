// Independent async module: never wait for the base template's deferred analytics.
// Company content is escaped, server-rendered HTML; no fetch or runtime geocoding.
const regions = [...document.querySelectorAll('.mdx-region')];
const unavailable = 'Map unavailable. Use the Open map links in the list.';

function popupFor(rows, selected) {
  const content = document.createElement('div');
  for (const row of rows) {
    const item = document.createElement('section');
    item.className = 'mdx-popup-company';
    item.setAttribute('aria-current', String(row === selected));
    const name = document.createElement('strong');
    name.textContent = `${row.querySelector('.mdx-number').textContent}: ${row.querySelector('h3').textContent}`;
    const location = document.createElement('p');
    location.textContent = row.querySelector('.mdx-location').textContent;
    const jobs = row.querySelector('.mdx-jobs').cloneNode(true);
    item.append(name, location, jobs);
    content.append(item);
  }
  return content;
}

function enhanceRegion(region, L) {
  const canvas = region.querySelector('.mdx-map');
  if (!canvas) return;
  const status = region.querySelector('.mdx-map-status');
  const rows = [...region.querySelectorAll('.mdx-entry[data-lat][data-lon]')];
  if (!rows.length) {
    status.textContent = 'No verified locations mapped yet. The company list is available below.';
    return;
  }
  const groups = new Map();
  for (const row of rows) {
    const point = [Number(row.dataset.lat), Number(row.dataset.lon)];
    if (!point.every(Number.isFinite) || Math.abs(point[0]) > 90 || Math.abs(point[1]) > 180) continue;
    const key = point.join(',');
    if (!groups.has(key)) groups.set(key, { point, rows: [] });
    groups.get(key).rows.push(row);
  }
  if (!groups.size) {
    status.textContent = unavailable;
    return;
  }
  canvas.hidden = false;
  const map = L.map(canvas, { scrollWheelZoom: false });
  // Coincident locations are grouped, never geographically offset or jittered.
  const bounds = [...groups.values()].map(group => group.point);
  const fit = () => map.fitBounds(bounds, { padding: [36, 36], maxZoom: 11, animate: false });
  fit();
  const tiles = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  });
  let failed = 0;
  let timeout;
  const tileUnavailable = () => {
    status.classList.remove('mdx-sr-only');
    status.textContent = 'Map unavailable: background tiles could not load. Numbered pins and Open map links are still available.';
  };
  tiles.on('loading', () => {
    failed = 0;
    status.textContent = 'Loading map tiles…';
    clearTimeout(timeout);
    timeout = setTimeout(tileUnavailable, 12000);
  });
  tiles.on('tileerror', () => { failed += 1; tileUnavailable(); });
  tiles.on('load', () => {
    clearTimeout(timeout);
    if (failed) tileUnavailable();
    else {
      status.classList.add('mdx-sr-only');
      status.textContent = 'Map ready. City pins are approximate; numbers match the list.';
    }
  });
  tiles.addTo(map);

  let returnFocus = null;
  map.on('popupclose', () => {
    const origin = returnFocus;
    returnFocus = null;
    if (origin) origin.focus();
  });
  canvas.addEventListener('keydown', event => {
    if (event.key === 'Escape') {
      event.preventDefault();
      event.stopPropagation();
      map.closePopup();
    }
  }, true);
  const markers = [];
  for (const group of groups.values()) {
    const numbers = document.createElement('span');
    numbers.textContent = group.rows.map(row => row.querySelector('.mdx-number').textContent).join(' · ');
    const label = group.rows.map(row => `${row.querySelector('.mdx-number').textContent}: ${row.querySelector('h3').textContent}`).join('; ');
    const marker = L.marker(group.point, {
      icon: L.divIcon({ className: 'mdx-marker', html: numbers, iconSize: null, iconAnchor: [16, 16] }),
      keyboard: true,
      title: label,
      riseOnHover: true,
    }).addTo(map);
    markers.push(marker);
    const icon = marker.getElement();
    icon.setAttribute('aria-label', label);
    icon.setAttribute('aria-pressed', 'false');
    marker.bindPopup(popupFor(group.rows, group.rows[0]), { maxWidth: 270, maxHeight: 180 });
    const select = (row, fromList = false) => {
      // Switching companies must not trigger the previous popup's return path.
      returnFocus = null;
      rows.forEach(candidate => candidate.removeAttribute('aria-current'));
      row.setAttribute('aria-current', 'true');
      markers.forEach(candidate => candidate.getElement().setAttribute('aria-pressed', String(candidate === marker)));
      marker.setPopupContent(popupFor(group.rows, row)).openPopup();
      if (fromList) {
        returnFocus = row.querySelector('.mdx-map-button');
        const selectedJobs = marker.getPopup().getElement().querySelector('.mdx-popup-company[aria-current="true"] .mdx-jobs');
        selectedJobs.focus();
        canvas.scrollIntoView({ block: 'center' });
      }
      // Marker-origin selection highlights the row without scrolling its popup away.
    };
    // Clear before Leaflet's built-in click toggle can close the old popup.
    marker.on('preclick', () => { returnFocus = null; });
    marker.on('click', () => select(group.rows[0]));
    // Leaflet's Enter handling opens the popup but does not emit marker click.
    // Handle both button keys here so keyboard activation also selects the row.
    icon.addEventListener('keydown', event => {
      if (event.key === ' ' || event.key === 'Enter') {
        event.preventDefault();
        event.stopPropagation();
        select(group.rows[0]);
      }
    });
    for (const row of group.rows) {
      const button = row.querySelector('.mdx-map-button');
      button.hidden = false;
      button.addEventListener('click', () => select(row, true));
    }
  }
  // Keep responsive maps fitted to verified coordinates, including mobile stacking.
  const observer = new ResizeObserver(() => {
    map.invalidateSize({ pan: false });
    fit();
  });
  observer.observe(canvas);
}

try {
  await import('../vendor/leaflet/leaflet.js');
  if (!window.L) throw new Error('Leaflet unavailable');
  for (const region of regions) {
    try { enhanceRegion(region, window.L); }
    catch {
      const status = region.querySelector('.mdx-map-status');
      if (status) status.textContent = unavailable;
      region.querySelectorAll('.mdx-map-button').forEach(button => { button.hidden = true; });
    }
  }
} catch {
  document.querySelectorAll('.mdx-map-status').forEach(status => { status.textContent = unavailable; });
}
