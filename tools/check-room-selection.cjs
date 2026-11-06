// Run by capture-demo.cjs --check-selection with its synthetic map and intercepted RPCs.
const assert = require('node:assert/strict');
module.exports = async function checkRoomSelection(page) {
  const quick = id => page.locator(`#roomquick .rq[data-seg="${id}"]`);
  const card = id => page.locator(`.preset[data-id="${id}"]`);
  const snapshot = () => page.evaluate(() => ({
    quick:[...document.querySelectorAll('#roomquick .rq.on')].map(b=>Number(b.dataset.seg)).sort(),
    markers:[...document.querySelectorAll('#markers .marker.sel')].map(b=>Number(b.dataset.seg)).sort(),
    preview:[...previewSegs].sort(),
    presets:[...presetSel],
    painted:[...new Map([[16,[120,100]],[17,[380,140]],[18,[550,150]],[19,[200,300]],[20,[660,410]]])]
      .filter(([,p])=>document.getElementById('roomhl').getContext('2d').getImageData(p[0],p[1],1,1).data[3]>0)
      .map(([id])=>id),
    body:selectionSources() ? mergePresets(selectionSources())?.body : {segments:orderedSelection()},
  }));
  const expectRooms = async ids => {
    const s=await snapshot(), expected=ids.slice().sort();
    assert.deepEqual(s.quick,expected,'room shortcuts');
    assert.deepEqual(s.markers,expected,'map markers');
    assert.deepEqual(s.preview,s.presets.length?expected:[],'preview targets');
    assert.deepEqual(s.painted,expected,'painted map highlight');
    assert.deepEqual([...new Set(s.body?.segments||[])].sort(),expected,'pending job');
  };
  // Give the two rooms different settings, to catch loss of per-room values.
  await page.evaluate(()=>{
    PRESETS.find(p=>p.id==='deep').body.per_room=[
      {id:19,fan:103,water:235,mop:300,repeat:2},
      {id:20,fan:104,water:240,mop:301,repeat:1},
    ];
  });
  const original=await page.evaluate(()=>JSON.stringify(PRESETS));
  await card('deep').click();
  await expectRooms([19,20]);
  await quick(19).click();
  await expectRooms([20]);
  assert.equal((await snapshot()).body.per_room[0].water,240);
  await page.locator('#fab_primary').click();
  assert.deepEqual(await page.evaluate(()=>pvBody.segments),[20]);
  assert.equal(await page.locator('#pv_del').isVisible(),false,'temporary changes must not delete the stored preset');
  await page.evaluate(()=>closePresetSheet());
  // Tap the floor itself, away from the marker, to deselect the remaining room.
  const point=await page.evaluate(()=>{
    const r=document.getElementById('map').getBoundingClientRect();
    return {x:r.left+r.width*660/800,y:r.top+r.height*410/560};
  });
  await page.mouse.click(point.x,point.y);
  await page.mouse.move(0,0);
  await expectRooms([]);
  assert.equal(await page.locator('#fab_primary').isDisabled(),true,'empty selection cannot become a full clean');
  await quick(16).click();
  await expectRooms([16]);
  await quick(16).click();
  await expectRooms([]);
  assert.equal(await card('deep').getAttribute('aria-pressed'),'false','last room removal unselects its preset');
  await card('deep').click();
  await quick(19).click();
  await expectRooms([20]);
  assert.equal((await snapshot()).body.per_room[0].water,240,'selecting the preset again restores its settings');
  await quick(16).click();
  await expectRooms([16,20]);
  await quick(16).click();
  await expectRooms([20]);
  // Excluding a shared room removes every occurrence across selected presets.
  await card('fast').click();
  await quick(20).click();
  await expectRooms([]);
  await quick(19).click();
  await expectRooms([19]);
  assert.deepEqual((await snapshot()).presets,[],'all empty room presets are unselected');
  // A refresh redraws the same selection, without restoring exclusions.
  await page.evaluate(()=>{renderRoomQuick(latest.rooms);renderMarkers(latest.overlays);renderPresets();});
  await expectRooms([19]);
  assert.equal(await page.evaluate(()=>JSON.stringify(PRESETS)),original,'saved presets stay unchanged');
  await page.evaluate(()=>{clearPresetSel();selected.clear();syncSel();});
  // An existing manual pick is not duplicated when a preset includes it.
  await quick(20).click();
  await card('fast').click();
  assert.deepEqual((await snapshot()).body.segments,[20]);
  await quick(20).click();
  await expectRooms([]);
  assert.equal(await card('fast').getAttribute('aria-pressed'),'false');
  await card('fast').click();
  await expectRooms([20]);
  await page.evaluate(()=>{clearPresetSel();selected.clear();syncSel();});
  // Zone-only presets keep their rectangles and do not select whole rooms.
  await card('deep').click();
  await page.locator('#fab_primary').click();
  await page.evaluate(()=>pvRemoveTarget(0));
  assert.deepEqual(await page.evaluate(()=>pvBody.segments),[20]);
  assert.deepEqual(await page.evaluate(()=>[...document.getElementById('roomhl').getContext('2d').getImageData(200,300,1,1).data].slice(0,3)),[255,255,255],
    'a room removed in the sheet must be veiled, not still highlighted by the preset');
  await page.evaluate(()=>{closePresetSheet();clearPresetSel();});
  await card('cooking').click();
  assert.deepEqual((await snapshot()).quick,[]);
  assert.equal((await snapshot()).body.zones.length,2);
  await page.evaluate(()=>clearPresetSel());
  await page.setViewportSize({width:390,height:844});
  await card('deep').click();
  await expectRooms([19,20]);
  await quick(19).click();
  await expectRooms([20]);
  await card('bath').click();
  await quick(20).click();
  await expectRooms([17]);
  assert.equal(await card('deep').getAttribute('aria-pressed'),'false');
  assert.equal(await card('bath').getAttribute('aria-pressed'),'true','other presets stay selected');
  assert.equal(await card('deep').locator('.pchev').textContent(),'›');
  await quick(17).click();
  await expectRooms([]);
  assert.equal(await card('bath').getAttribute('aria-pressed'),'false');
  assert.equal(await page.locator('#fab_primary').isDisabled(),true);
  console.log('Room selection checks passed: shortcuts, map taps, exclusions, settings, empty guard, refresh and zones.');
};
