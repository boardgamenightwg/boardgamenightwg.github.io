// Independent async module: never wait for the base template's deferred analytics.
// Company content is escaped, server-rendered HTML; no fetch or runtime geocoding.
const regions = [...document.querySelectorAll('.mdx-region')].sort((a, b) =>
  Number(b.id === 'mdx-region-boston') - Number(a.id === 'mdx-region-boston'));
let leaflet;
const enhanced = new Set();
function enhanceSelected() {
  const region = regions.find(region => !region.hidden);
  if (!leaflet || !region || enhanced.has(region)) return;
  enhanced.add(region);
  try { enhanceRegion(region, leaflet); }
  catch {
    const status = region.querySelector('.mdx-map-status');
    if (status) status.textContent = unavailable;
    region.querySelectorAll('.mdx-map-button').forEach(button => { button.hidden = true; });
  }
}

// Tabs work independently of the optional map library. No JS leaves all lists visible.
if (regions.length) {
  const tabs = document.createElement('div');
  tabs.className = 'mdx-region-tabs';
  tabs.setAttribute('role', 'tablist');
  tabs.setAttribute('aria-label', 'Directory region');
  const buttons = regions.map(region => {
    const button = document.createElement('button');
    button.type = 'button';
    button.id = `${region.id}-tab`;
    button.setAttribute('role', 'tab');
    button.setAttribute('aria-controls', region.id);
    button.setAttribute('aria-label', region.dataset.regionLabel);
    const icon = document.createElement('span');
    icon.setAttribute('aria-hidden', 'true');
    icon.textContent = region.id === 'mdx-region-boston' ? '🫘🌆 ' : region.id === 'mdx-region-bay' ? '🌉🌅 ' : '';
    const count = document.createElement('span');
    count.className = 'mdx-tab-count';
    count.setAttribute('aria-hidden', 'true');
    count.textContent = String(region.querySelectorAll('.mdx-entry').length);
    button.append(icon, region.dataset.regionLabel, count);
    region.setAttribute('role', 'tabpanel');
    region.setAttribute('aria-labelledby', button.id);
    region.tabIndex = 0;
    tabs.append(button);
    return button;
  });
  const activate = index => {
    regions.forEach((region, i) => {
      region.hidden = i !== index;
      buttons[i].setAttribute('aria-selected', String(i === index));
      buttons[i].tabIndex = i === index ? 0 : -1;
    });
    enhanceSelected();
  };
  buttons.forEach((button, index) => {
    button.addEventListener('click', () => activate(index));
    button.addEventListener('keydown', event => {
      let next;
      if (event.key === 'ArrowRight') next = (index + 1) % buttons.length;
      else if (event.key === 'ArrowLeft') next = (index + buttons.length - 1) % buttons.length;
      else if (event.key === 'Home') next = 0;
      else if (event.key === 'End') next = buttons.length - 1;
      else return;
      event.preventDefault();
      activate(next);
      buttons[next].focus();
    });
  });
  document.querySelector('.mdx-intro').after(tabs);
  activate(0);
}
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
    const careers = row.querySelector('.mdx-jobs');
    const jobs = (careers || row.querySelector('h3 a')).cloneNode(true);
    if (!careers) jobs.textContent = 'Website →';
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
        const selectedJobs = marker.getPopup().getElement().querySelector('.mdx-popup-company[aria-current="true"] a');
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
    if (!canvas.clientWidth || !canvas.clientHeight) return;
    map.invalidateSize({ pan: false });
    fit();
    // Refitting can move an open popup outside the clipped map; re-run its auto-pan.
    map.eachLayer(layer => {
      if (layer instanceof L.Popup && layer.isOpen()) {
        const focus = layer.getElement().contains(document.activeElement) ? document.activeElement : null;
        layer.update();
        // Leaflet rebuilds popup contents during update, temporarily detaching focus.
        if (focus) focus.focus({ preventScroll: true });
      }
    });
  });
  observer.observe(canvas);
}

try {
  await import('../vendor/leaflet/leaflet.js');
  if (!window.L) throw new Error('Leaflet unavailable');
  leaflet = window.L;
  enhanceSelected();
} catch {
  document.querySelectorAll('.mdx-map-status').forEach(status => { status.textContent = unavailable; });
}
