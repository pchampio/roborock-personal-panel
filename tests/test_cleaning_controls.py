"""Cleaning recovery and dock care with fake RPCs; no device connection."""
import asyncio
from datetime import datetime as RealDatetime
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock, Mock, patch

sys.path.insert(0, str(Path(__file__).parents[1] / 'app'))
import app as m


class Resume(unittest.IsolatedAsyncioTestCase):
    async def test_refresh_selects_held_job_without_fallback(self):
        for kind, command in ((1, 'app_start'), (2, 'resume_zoned_clean'),
                              (3, 'resume_segment_clean'), (4, 'app_resume_build_map')):
            with self.subTest(kind=kind):
                st = NS(state=12, in_cleaning=0, in_returning=0)
                st.refresh = AsyncMock(side_effect=lambda: setattr(st, 'in_cleaning', kind))
                props = NS(status=st, command=NS(send=AsyncMock(return_value=['ok'])))
                await m._resume_loaded_job(props)
                st.refresh.assert_awaited_once()
                self.assertEqual(props.command.send.await_args.args[0].value, command)

    async def test_refused_room_resume_never_starts_full_clean(self):
        props = NS(status=NS(state=10, in_cleaning=3, refresh=AsyncMock()),
                   command=NS(send=AsyncMock(side_effect=RuntimeError('refused'))))
        with self.assertRaises(m.web.HTTPBadGateway):
            await m._resume_loaded_job(props)
        props.command.send.assert_awaited_once_with(m.RoborockCommand.RESUME_SEGMENT_CLEAN)

    async def test_unknown_or_missing_job_and_already_cleaning_send_nothing(self):
        for kind, state in ((0, 10), (None, 12), (99, 10), (3, 18)):
            props = NS(status=NS(state=state, in_cleaning=kind, refresh=AsyncMock()),
                       command=NS(send=AsyncMock()))
            with self.assertRaises(m.web.HTTPConflict):
                await m._resume_loaded_job(props)
            props.command.send.assert_not_awaited()

    async def test_paused_return_continues_docking(self):
        props = NS(status=NS(state=10, in_cleaning=0, in_returning=1, refresh=AsyncMock()),
                   command=NS(send=AsyncMock()))
        await m._resume_loaded_job(props)
        props.command.send.assert_awaited_once_with(m.RoborockCommand.APP_CHARGE)

    async def test_toolbar_and_history_share_resume_logic(self):
        props = NS(status=NS(refresh=AsyncMock(), error_code=0),
                   map_content=NS(refresh=AsyncMock()))
        state = NS(_lock=asyncio.Lock(), props=AsyncMock(return_value=props),
                   selected_duid='test', device=Mock(return_value=None))
        tracker = NS(pending=Mock(return_value=None), last_launch=Mock(return_value=None))
        req = NS(json=AsyncMock(return_value={'action': 'resume'}), can_read_body=True)
        plan = {'available': True, 'kind': 'vacuum', 'modes': {}, 'blocked_by_error': False}
        with patch.object(m, 'STATE', state), patch.object(m, 'PATROL', NS(active=False)), \
             patch.object(m, 'TRACKER', tracker), patch.object(m, '_resume_plan', return_value=plan), \
             patch.object(m, '_journal'), patch.object(m, '_resume_loaded_job', AsyncMock(return_value=['ok'])) as resume:
            await m.handle_command(req)
            await m.handle_resume(req)
            self.assertEqual(resume.await_count, 2)


class Care(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.journal = Path(self.tmp.name) / 'journal.jsonl'
        self.journal.touch()
        self.care = dict(run=None, phase='idle', washed=False, dried=False, emptied=False,
                         dust_skipped=False, dust_attempted=False, since=1, tries={}, note='')
        self.st = NS(state=8, in_cleaning=0, error_code=0, clean_time=400, dry_status=0,
                     wash_phase=0, refresh=AsyncMock())
        self.props = NS(status=self.st, command=NS(send=AsyncMock(return_value=['ok'])))
        self.rec = dict(begin=100, end=600, duration_sec=400, clean_type=3,
                        complete=0, error=0, mopped=True, wash_count=1)
        patches = dict(STATE=NS(_lock=asyncio.Lock(), devices=['test'], selected_duid='test',
                               props=AsyncMock(return_value=self.props)),
                       PATROL=NS(active=False), _mop_care=self.care, _care_cfg={'armed_at':0},
                       LAUNCH_JOURNAL=str(self.journal), _LAST_DOCK_BUSY=None,
                       RUNS=NS(for_run=Mock(return_value=None)),
                       _last_run=AsyncMock(return_value=self.rec), _care_dnd=AsyncMock(return_value=False))
        for name, value in patches.items():
            p = patch.object(m, name, value); p.start(); self.addCleanup(p.stop)
        p = patch.object(m.time, 'time', return_value=1000); p.start(); self.addCleanup(p.stop)

    def event(self, at, to, clean_time=400, was=()):
        self.row(at, 'dock_busy', {'to':to, 'from':list(was), 'clean_time':clean_time})

    def row(self, at, kind, payload):
        with self.journal.open('a') as f:
            f.write(json.dumps({'t':at, 'kind':kind, 'payload':payload})+'\n')

    def evidence(self):
        return m._care_evidence(100, 600, 400)

    def calls(self):
        return [c.args[0].value for c in self.props.command.send.await_args_list]

    async def test_mid_run_wash_and_later_drying_are_not_final_care(self):
        self.event(300, ['washing'], 100)
        self.event(610, ['drying'])
        self.assertEqual(self.evidence(), (False, False, False, False, False))

    async def test_final_wash_before_record_end_counts_across_restart(self):
        self.event(590, ['washing'])
        self.event(610, ['drying'])
        self.assertEqual(self.evidence(), (True, True, False, False, False))

    async def test_new_wash_requires_new_drying(self):
        self.event(610, ['washing'])
        self.event(620, ['drying'])
        self.event(700, ['washing'])
        self.assertEqual(self.evidence()[:2], (True, False))

    async def test_manual_empty_is_not_repeated(self):
        self.row(650, 'command', {'method':'app_start_collect_dust','result':['ok']})
        self.event(700, ['washing']); self.event(800, ['drying'])
        await m._mop_care_tick()
        self.assertEqual(self.calls(), [])
        self.assertEqual(self.care['phase'], 'done')

    async def test_empty_then_wash_even_without_saved_request(self):
        self.event(300, ['washing'], 100); self.event(610, ['drying'])
        await m._mop_care_tick()
        await m._mop_care_tick()
        self.assertEqual(self.calls(), ['app_start_collect_dust', 'app_start_wash'])
        await m._mop_care_tick()  # no immediate repeat while waiting for wash to start
        self.assertEqual(len(self.calls()), 2)

    async def test_dnd_skips_empty_even_after_restart_but_allows_mop_care(self):
        m._care_dnd.return_value = True
        await m._mop_care_tick()
        self.assertEqual(self.calls(), ['app_start_wash'])
        self.care.update(run=None, phase='idle')
        m._care_dnd.return_value = False
        self.event(1010, ['washing']); self.event(1020, ['drying'])
        await m._mop_care_tick()
        self.assertEqual(self.calls(), ['app_start_wash'])
        self.assertTrue(self.care['dust_skipped'])

    async def test_failed_empty_is_not_retried_or_reported_as_done(self):
        self.props.command.send.side_effect = RuntimeError('timeout')
        await m._mop_care_tick()
        self.props.command.send.side_effect = None
        await m._mop_care_tick()
        self.assertEqual(self.calls(), ['app_start_collect_dust', 'app_start_wash'])
        self.assertFalse(self.care['emptied'])
        self.assertTrue(self.care['dust_attempted'])

    async def test_vacuum_only_gets_dust_empty_not_wash(self):
        self.rec.update(mopped=False, wash_count=0)
        await m._mop_care_tick(); await m._mop_care_tick()
        self.assertEqual(self.calls(), ['app_start_collect_dust'])

    async def test_mop_only_does_not_empty_dust(self):
        m.RUNS.for_run.return_value = {'requested': {'fan':106, 'water':203}}
        await m._mop_care_tick()
        self.assertEqual(self.calls(), ['app_start_wash'])

    async def test_no_care_away_from_dock_during_fault_job_route_or_wash(self):
        for fields in ({'state':3}, {'state':6}, {'state':23}, {'state':22},
                       {'in_cleaning':3}, {'error_code':54}, {'wash_phase':11}):
            with self.subTest(fields=fields), patch.multiple(self.st, **fields):
                self.care['since'] = 1
                await m._mop_care_tick()
        with patch.object(m.PATROL, 'active', True):
            self.care['since'] = 1
            await m._mop_care_tick()
        self.assertEqual(self.calls(), [])

    async def test_completed_old_and_empty_jobs_are_left_alone(self):
        for update in ({'complete':1}, {'end':1}, {'duration_sec':0}, {'clean_type':4}):
            with self.subTest(update=update), patch.dict(self.rec, update):
                self.care.update(run=None, phase='idle')
                with patch.object(m, 'MOP_CARE_MAX_AGE', 900):
                    await m._mop_care_tick()
        self.assertEqual(self.calls(), [])

    async def test_already_washing_is_not_restarted(self):
        self.st.wash_phase = 11
        await m._mop_care_tick()
        self.assertEqual(self.calls(), [])


class DND(unittest.IsolatedAsyncioTestCase):
    async def check_time(self, hour, minute=0, start=22, end=8):
        dnd = NS(enabled=1, start_hour=start, start_minute=0, end_hour=end,
                 end_minute=0, refresh=AsyncMock())
        props = NS(dnd=dnd, command=NS(send=AsyncMock(return_value=['Europe/Paris'])))
        class Clock:
            @staticmethod
            def now(zone):
                self.assertEqual(str(zone), 'Europe/Paris')
                return RealDatetime(2026, 1, 1, hour, minute, tzinfo=zone)
        with patch.object(m, 'datetime', Clock):
            return await m._care_dnd(props)

    async def test_overnight_schedule_boundaries_use_robot_timezone(self):
        for hour, minute, expected in ((21,59,False),(22,0,True),(0,0,True),(7,59,True),(8,0,False)):
            with self.subTest(hour=hour, minute=minute):
                self.assertEqual(await self.check_time(hour,minute), expected)

    async def test_daytime_and_equal_bounds(self):
        self.assertTrue(await self.check_time(14, start=13, end=15))
        self.assertFalse(await self.check_time(15, start=13, end=15))
        self.assertTrue(await self.check_time(12, start=8, end=8))

    async def test_disabled_and_unavailable_schedule(self):
        props = NS(dnd=NS(enabled=0, refresh=AsyncMock()), command=NS(send=AsyncMock()))
        self.assertFalse(await m._care_dnd(props))
        props.command.send.assert_not_awaited()
        props.dnd.refresh.side_effect = RuntimeError('offline')
        self.assertTrue(await m._care_dnd(props))


class ResumePlan(unittest.TestCase):
    def test_held_zone_job_wins_over_previous_run_reconstruction(self):
        st = NS(state=12, error_code=0, in_cleaning=2)
        runs = NS(last_run=Mock(return_value=None), resume_dismissed=Mock(return_value=False),
                  for_run=Mock(return_value=None))
        with patch.object(m, 'RUNS', runs), \
             patch.object(m, 'TRACKER', NS(pending=Mock(return_value=None), last_launch=Mock(return_value=None))), \
             patch.object(m, '_modes', return_value={}), \
             patch.object(m, '_last_run_cached', return_value={'begin':1,'end':2,'clean_type':2}), \
             patch.object(m, '_zone_resume') as reconstruct:
            plan = m._resume_plan('test', st)
            self.assertEqual(plan['kind'], 'vacuum')
            reconstruct.assert_not_called()


if __name__ == '__main__':
    unittest.main()
