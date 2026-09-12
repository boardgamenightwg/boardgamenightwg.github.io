// Pure data/state functions shared by the browser and Node's built-in tests.
export const relationshipTypes = Object.freeze({
  shares_events_from: {outgoing: 'Shares events from →', incoming: 'Events shared by ←', symmetric: false},
  runs_activities_with: {outgoing: 'Runs activities with ↔', incoming: 'Runs activities with ↔', symmetric: true},
  operates: {outgoing: 'Operates →', incoming: 'Operated by ←', symmetric: false},
});

export function safeUrl(value) {
  if (typeof value !== 'string' || !/^https?:\/\//i.test(value) || /[\s\\\u0000-\u001f\u007f<>"]/u.test(value)) return null;
  try {
    const url = new URL(value);
    return ['https:', 'http:'].includes(url.protocol) && url.hostname && !url.username && !url.password && url.port !== '0'
      ? url.href : null;
  } catch { return null; }
}

export function visibleNodes(data, state) {
  const query = state.query.trim().toLocaleLowerCase();
  return data.nodes.filter(node => node.region === state.region
    && (state.category === 'all' || node.category === state.category)
    && [node.name, node.summary, node.participation, node.category].join(' ').toLocaleLowerCase().includes(query));
}

export function visibleEdges(data, nodes) {
  const ids = new Set(nodes.map(node => node.id));
  return data.edges.filter(edge => ids.has(edge.source) && ids.has(edge.target));
}

export function reconcile(data, state) {
  const nodes = visibleNodes(data, state);
  return {...state, selected: nodes.some(node => node.id === state.selected) ? state.selected : nodes[0]?.id ?? null};
}

export function follow(data, state, id) {
  const target = data.nodes.find(node => node.id === id);
  return target ? {region: target.region, category: 'all', query: '', selected: id} : reconcile(data, state);
}

export function relationships(data, id) {
  return data.edges.filter(edge => edge.source === id || edge.target === id).map(edge => ({
    edge,
    other: data.nodes.find(node => node.id === (edge.source === id ? edge.target : edge.source)),
    label: relationshipTypes[edge.type][edge.source === id ? 'outgoing' : 'incoming'],
  }));
}

// Two staggered columns scale vertically instead of shrinking labels or requiring coordinates.
// Stable dataset order keeps nodes in place while filtering.
export function layout(nodes) {
  return nodes.map((node, index) => ({id: node.id, x: index % 2 ? 75 : 25, y: 100 + Math.floor(index / 2) * 200}));
}
