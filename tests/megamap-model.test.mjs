import {test} from 'node:test';
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
const path = new URL('../static/scripts/megamap-model.mjs', import.meta.url);
const data = {nodes: [
  {id:'a', name:'Alpha', region:'boston', category:'robotics', summary:'Building robots', participation:'Join'},
  {id:'b', name:'Beta', region:'boston', category:'making', summary:'Making robots', participation:'Visit'},
  {id:'c', name:'Gamma', region:'bay', category:'robotics', summary:'Robots', participation:'Join'}
], edges:[{source:'a',target:'b',type:'shares_events_from'}, {source:'c',target:'a',type:'operates'}, {source:'b',target:'c',type:'runs_activities_with'}]};
async function model() {
  assert.ok(existsSync(path), 'Missing pure Megamap model');
  return import(path);
}
test('search AND category AND region, including empty results', async () => {
  const {visibleNodes} = await model();
  assert.deepEqual(visibleNodes(data, {region:'boston',category:'robotics',query:' ROBOTS '}).map(n=>n.id), ['a']);
  assert.deepEqual(visibleNodes(data, {region:'boston',category:'making',query:'Alpha'}), []);
});
test('selection is cleared when hidden, first result selected, empty selection is null', async () => {
  const {reconcile} = await model();
  assert.equal(reconcile(data,{region:'boston',category:'making',query:'',selected:'a'}).selected,'b');
  assert.equal(reconcile(data,{region:'boston',category:'all',query:'xxx',selected:'a'}).selected,null);
});
test('following a relationship reveals target across filters and regions', async () => {
  const {follow} = await model();
  assert.deepEqual(follow(data,{region:'boston',category:'making',query:'Beta',selected:'b'},'c'),
    {region:'bay',category:'all',query:'',selected:'c'});
});
test('visible edges require both endpoints and preserve direction', async () => {
  const {visibleEdges} = await model();
  assert.deepEqual(visibleEdges(data,[data.nodes[0],data.nodes[1]]),[data.edges[0]]);
});
test('incoming, outgoing and symmetric relationship labels', async () => {
  const {relationships} = await model();
  const rel = relationships(data,'a');
  assert.equal(rel[0].label,'Shares events from →');
  assert.equal(rel[0].other.id,'b');
  assert.equal(rel[1].label,'Operated by ←');
  assert.equal(relationships(data,'b')[0].label,'Events shared by ←');
  assert.equal(relationships(data,'c')[1].label,'Runs activities with ↔');
});
test('runtime URL allowlist rejects executable, relative, malformed and credentialed URLs', async () => {
  const {safeUrl} = await model();
  for (const value of ['javascript:alert(1)','data:text/html,hi','//example.org','https://','https://a:b@example.org','https://example.org/\nfoo','https:example.org','https://example.org:0/']) assert.equal(safeUrl(value), null);
  assert.equal(safeUrl('https://example.org/'),'https://example.org/');
});
test('automatic positions are deterministic, unique and require no node coordinates', async () => {
  const {layout} = await model();
  const result = layout(data.nodes);
  assert.deepEqual(result, layout(data.nodes));
  assert.equal(new Set(result.map(p=>`${p.x},${p.y}`)).size, 3);
  assert.ok(result.every(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)));
});
