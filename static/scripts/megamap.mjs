import {visibleNodes, visibleEdges, reconcile, follow, relationships, layout, safeUrl, relationshipTypes} from './megamap-model.mjs';

const root = document.querySelector('#megamap');
const $ = id => root.querySelector(`#${id}`);
const element = (tag, text, className) => {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
};
const svgElement = (tag, attributes = {}) => {
  const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
  for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
  return node;
};

function sourceList(sources) {
  const list = element('ul', undefined, 'mm-sources');
  for (const source of sources) {
    const url = safeUrl(source.url);
    if (!url) continue;
    const link = element('a', source.label);
    link.href = url;
    link.rel = 'noopener noreferrer';
    const item = element('li');
    item.append(link);
    list.append(item);
  }
  return list;
}

function enhance(data) {
  let state = {region: Object.hasOwn(data.regions, 'boston') ? 'boston' : Object.keys(data.regions)[0], category: 'all', query: '', selected: null};
  let announceTimer;
  const graph = root.querySelector('.mm-graph');
  const nodes = new Map();
  for (const [id, label] of Object.entries(data.regions)) $('mm-region').append(new Option(label, id));
  $('mm-category').append(new Option('All categories', 'all'));
  for (const [id, label] of Object.entries(data.categories)) $('mm-category').append(new Option(label, id));

  for (const node of data.nodes) {
    const li = element('li');
    const button = element('button', undefined, 'mm-node');
    button.type = 'button';
    button.dataset.node = node.id;
    button.setAttribute('aria-controls', 'mm-detail');
    button.append(element('strong', node.name), element('span', data.categories[node.category]),
      element('small', node.status === 'seed' ? 'Seed · research pending' : node.status));
    button.addEventListener('click', () => {
      state.selected = node.id;
      render();
      if (matchMedia('(max-width: 1000px)').matches) {
        const heading = $('mm-detail').querySelector('h2');
        heading.focus();
        heading.scrollIntoView({block: 'start'});
      }
    });
    li.append(button);
    $('mm-nodes').append(li);
    nodes.set(node.id, {li, button});
  }

  function details() {
    const detail = $('mm-detail');
    const node = data.nodes.find(item => item.id === state.selected);
    detail.replaceChildren();
    const title = element('h2', node?.name ?? 'No organization selected');
    title.tabIndex = -1;
    detail.append(title);
    const back = element('a', 'Back to organizations', 'mm-back');
    back.href = '#mm-network-title';
    detail.append(back);
    if (!node) {
      detail.append(element('p', 'Adjust your search or reset filters to see organizations.'));
      return;
    }
    detail.append(element('span', `${data.categories[node.category]} · ${node.status}`, 'mm-badge'),
      element('p', node.summary), element('h3', 'Taking part'), element('p', node.participation),
      element('p', node.status === 'seed' ? 'Research pending · not yet verified' : `Last verified: ${node.last_verified}`, 'mm-meta'),
      sourceList(node.sources), element('h3', 'Recorded connections'));
    const connections = relationships(data, node.id);
    if (!connections.length) detail.append(element('p', 'No sourced relationships recorded. This is not a claim of isolation.'));
    for (const {edge, other, label} of connections) {
      const entry = element('section', undefined, 'mm-relationship');
      entry.append(element('p', label, 'mm-relation-label'));
      const button = element('button', other.name, 'mm-follow');
      button.type = 'button';
      button.dataset.follow = other.id;
      button.addEventListener('click', () => {
        state = follow(data, state, other.id);
        render();
        // Only explicit relationship navigation moves focus, never typing/filter updates.
        $('mm-detail').querySelector('h2').focus();
      });
      entry.append(button, element('p', edge.description),
        element('p', `${edge.status} · Last verified: ${edge.last_verified}`, 'mm-meta'), sourceList(edge.sources));
      detail.append(entry);
    }
  }

  function drawEdges() {
    const svg = $('mm-edges');
    const width = graph.clientWidth;
    if (!width || graph.hidden) return; // ResizeObserver redraws once the workspace is visible.
    const regionNodes = data.nodes.filter(node => node.region === state.region);
    const positions = new Map(layout(regionNodes).map(point => [point.id, {x: point.x * width / 100, y: point.y}]));
    const height = Math.max(200, Math.ceil(regionNodes.length / 2) * 200);
    graph.style.setProperty('--mm-graph-height', `${height}px`);
    svg.setAttribute('viewBox', `0 0 ${width} ${height}`);
    svg.replaceChildren();
    const defs = svgElement('defs');
    for (const type of Object.keys(relationshipTypes)) {
      const marker = svgElement('marker', {id: `mm-arrow-${type}`, viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse'});
      marker.append(svgElement('path', {d: 'M 0 0 L 10 5 L 0 10 z', class: `mm-arrow ${type}`}));
      defs.append(marker);
    }
    svg.append(defs);
    for (const edge of visibleEdges(data, visibleNodes(data, state))) {
      const from = positions.get(edge.source), to = positions.get(edge.target);
      const dx = to.x - from.x, dy = to.y - from.y;
      // Clip lines to the card boundary, so arrowheads remain visible outside nodes.
      const clip = 1 / Math.max(Math.abs(dx) / 100, Math.abs(dy) / 78);
      const line = svgElement('line', {x1: from.x + dx * clip, y1: from.y + dy * clip,
        x2: to.x - dx * clip, y2: to.y - dy * clip,
        class: `mm-edge ${edge.type}${edge.source === state.selected || edge.target === state.selected ? ' mm-highlight' : ''}`});
      line.setAttribute('marker-end', `url(#mm-arrow-${edge.type})`);
      if (relationshipTypes[edge.type].symmetric) line.setAttribute('marker-start', `url(#mm-arrow-${edge.type})`);
      svg.append(line);
    }
  }

  function render() {
    state = reconcile(data, state);
    const visible = visibleNodes(data, state);
    const ids = new Set(visible.map(node => node.id));
    $('mm-region').value = state.region;
    $('mm-category').value = state.category;
    $('mm-search').value = state.query;
    $('mm-network-title').textContent = `${data.regions[state.region]} community network`;
    const positions = new Map(layout(data.nodes.filter(node => node.region === state.region)).map(point => [point.id, point]));
    for (const [id, {li, button}] of nodes) {
      li.hidden = !ids.has(id);
      button.setAttribute('aria-pressed', String(state.selected === id));
      const point = positions.get(id);
      if (point) {
        li.style.setProperty('--mm-x', `${point.x}%`);
        li.style.setProperty('--mm-y', `${point.y}px`);
      }
    }
    graph.hidden = visible.length === 0;
    $('mm-empty').hidden = visible.length !== 0;
    details();
    drawEdges();
    clearTimeout(announceTimer);
    announceTimer = setTimeout(() => {
      const selected = data.nodes.find(node => node.id === state.selected);
      $('mm-status').textContent = `${visible.length} organizations · ${visibleEdges(data, visible).length} visible connections${selected ? ` · Selected: ${selected.name}` : ' · No matches'}`;
    }, 200);
  }

  $('mm-search').addEventListener('input', event => { state.query = event.target.value; render(); });
  $('mm-category').addEventListener('change', event => { state.category = event.target.value; render(); });
  $('mm-region').addEventListener('change', event => {
    state = {region: event.target.value, category: 'all', query: '', selected: null};
    render();
  });
  $('mm-reset').addEventListener('click', () => { state = {...state, query: '', category: 'all'}; render(); });
  render();
  $('mm-app').hidden = false;
  $('mm-directory').open = false;
  $('mm-load-status').hidden = true;
  new ResizeObserver(drawEdges).observe(graph);
}

// Canonical data is fetched, never interpolated into an inline script or HTML string.
try {
  const response = await fetch(root.dataset.source);
  if (!response.ok) throw new Error(`Data request returned ${response.status}`);
  enhance(await response.json());
} catch (error) {
  $('mm-app').hidden = true;
  $('mm-directory').open = true;
  $('mm-load-status').hidden = false;
  $('mm-load-status').textContent = 'Interactive map unavailable. Use the full readable directory below.';
  console.warn('Megamap enhancement unavailable:', error.message);
}
