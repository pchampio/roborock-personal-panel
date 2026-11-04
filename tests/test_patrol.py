"""Route sequencing checks with a fake robot; never connect to hardware."""
import asyncio
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import AsyncMock, patch

spec = importlib.util.spec_from_file_location("patrol", Path(__file__).parents[1]/"app"/"patrol.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Robot:
    def __init__(self):
        self.calls = []
        self.state = 3
        self.pos = [1000, 1000]
        self.target = None
        self.arrive = True
        self.error = 0
        self.flag = 0
        self.manual = False
        self.dock_approach = [1500, 1500]

    async def send(self, duid, method, params):
        self.calls.append((method, params))
        if method == "app_goto_target":
            self.target = params
            self.state = 16
        elif method == "app_charge":
            self.state = 6
        elif method == "app_rc_start":
            self.state = 7
        elif method in ("app_rc_end", "app_stop"):
            self.state = 3
        return ["ok"]

    async def snapshot(self, duid, position):
        result = {"state": self.state, "map_flag": self.flag, "error": self.error, "in_cleaning": 0}
        if self.state == 16 and self.arrive:
            self.pos = self.target
            self.state = 3
        if self.state == 6 and self.arrive:
            self.state = 8
        if position:
            result['position'] = self.pos
            result['dock_approach'] = self.dock_approach
        return result


class Routes(unittest.IsolatedAsyncioTestCase):
    async def test_camera_lock_after_rotation_releases_and_continues(self):
        class CameraRobot(Robot):
            locked = False

            async def send(self, duid, method, params):
                if method == 'app_goto_target' and self.locked:
                    raise ValueError('-10003 action locked')
                result = await super().send(duid, method, params)
                if method == 'app_rc_end': self.locked = True
                if method == 'stop_camera_preview': self.locked = False
                return result

            async def snapshot(self, duid, position):
                return {**await super().snapshot(duid, position), 'home_sec_status': int(self.locked)}

        r=CameraRobot();p=self.runner(r)
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000},{'kind':'turn','degrees':90},
                               {'kind':'goto','x':3000,'y':1000},{'kind':'home'}],'scan')
        await self.finish(p)
        methods=[method for method,_ in r.calls]
        self.assertEqual(p.state['phase'],'complete')
        self.assertEqual(p.state['camera_revision'],1)
        self.assertEqual(methods.count('app_goto_target'),3)
        self.assertLess(methods.index('stop_camera_preview'),methods.index('app_goto_target',1))
        self.assertIn('app_charge',methods)

    async def test_cleanup_failure_preserves_original_error(self):
        class FailedRobot(Robot):
            async def send(self, duid, method, params):
                if method == 'app_goto_target': raise ValueError('Original navigation failure')
                if method == 'app_stop': raise ValueError('Stop also failed')
                return await super().send(duid, method, params)

        r=FailedRobot();p=self.runner(r)
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000}],'failure')
        await self.finish(p)
        self.assertIn('Original navigation failure',p.state['message'])
        self.assertEqual(p.state['cleanup_errors'],['app_stop'])

    async def test_non_camera_lock_is_bounded_and_never_changes_child_lock(self):
        class LockedRobot(Robot):
            async def send(self, duid, method, params):
                if method == 'app_goto_target': raise ValueError('-10003 action locked')
                return await super().send(duid, method, params)

        r=LockedRobot();p=self.runner(r);p.UNLOCK_TIMEOUT=.005
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000}],'locked')
        await self.finish(p)
        self.assertEqual(p.state['phase'],'error')
        self.assertEqual([m for m,_ in r.calls],['app_stop','stop_camera_preview'])

    async def test_rotation_speed_changes_rate_and_preserves_angle(self):
        for degrees in (-360, 360):
            for speed in (.5, 1, 2):
                r = Robot(); p = m.Patrol(r.send, r.snapshot)
                p.duid = 'robot'; p.map_flag = 0
                with patch.object(m.asyncio, 'sleep', new_callable=AsyncMock):
                    await p._turn({'kind':'turn','degrees':degrees,'speed':speed})
                moves = [params[0] for method, params in r.calls if method == 'app_rc_move']
                self.assertTrue(all(move['omega'] == (.6*speed if degrees < 0 else -.6*speed) for move in moves))
                angle = sum(abs(move['omega'])*move['duration']/1000 for move in moves)
                self.assertAlmostEqual(angle, m.math.radians(abs(degrees)), delta=.12)
                self.assertEqual(r.calls[-1][0], 'app_rc_end')

    def runner(self, robot):
        p = m.Patrol(robot.send, robot.snapshot)
        p.POLL = .001
        p.PAIR_DELAY = .001
        p.OMEGA = 1000  # no physical rotation; keep the sequence test quick
        p.GOTO_TIMEOUT = .05
        p.HOME_TIMEOUT = .05
        return p

    async def test_waypoints_rotation_and_session_end(self):
        r=Robot();p=self.runner(r)
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000},{'kind':'turn','degrees':90},{'kind':'goto','x':3000,'y':1000}],'route')
        await self.finish(p)
        methods=[c[0] for c in r.calls]
        self.assertEqual(methods,['app_goto_target','app_rc_start','app_rc_move','app_rc_end','app_goto_target','stop_camera_preview'])
        self.assertEqual(p.state['phase'],'complete')
        self.assertEqual(r.calls[2][1][0]['velocity'],0)

    async def ack_close(self, p):
        while p.active and not p.state.get('camera_close_requested'):
            await asyncio.sleep(.001)
        if p.active:
            p.camera_closed(p.token)

    async def finish(self, p):
        browser = asyncio.create_task(self.ack_close(p))
        try:
            await p.task
        finally:
            browser.cancel()
            await asyncio.gather(browser, return_exceptions=True)

    async def test_home_approaches_then_closes_session_before_docking(self):
        r=Robot();p=self.runner(r)
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000},{'kind':'home'}],'home')
        await self.finish(p)
        self.assertEqual([c[0] for c in r.calls],['app_goto_target','app_goto_target','app_rc_end','stop_camera_preview','app_charge'])
        self.assertEqual(r.calls[1][1],r.dock_approach)
        self.assertEqual(r.state,8)
        self.assertTrue(p.state['camera_closed'])
        self.assertEqual(p.state['phase'],'complete')

    async def test_stop_while_returning_home(self):
        class ReturningRobot(Robot):
            async def send(self, duid, method, params):
                if method=='app_charge': self.arrive=False
                return await super().send(duid,method,params)
        r=ReturningRobot();p=self.runner(r)
        token=await p.start('robot',0,[{'kind':'home'}],'home')
        await self.ack_close(p)
        while r.state!=6: await asyncio.sleep(.001)
        await p.stop(token)
        self.assertEqual([c[0] for c in r.calls],['app_goto_target','app_rc_end','stop_camera_preview','app_charge','app_stop'])
        self.assertEqual(p.state['phase'],'stopped')

    async def test_docking_waits_for_browser_acknowledgement(self):
        r=Robot();r.pos=r.dock_approach;p=self.runner(r)
        token=await p.start('robot',0,[{'kind':'home'}],'home')
        await asyncio.sleep(.005)
        self.assertTrue(p.state['camera_close_requested'])
        self.assertEqual(r.calls,[])
        with self.assertRaises(ValueError):p.camera_closed('stale')
        self.assertFalse(p.browser_closed.is_set())
        p.camera_closed(token)
        await p.task
        self.assertEqual([m for m,_ in r.calls],['app_rc_end','stop_camera_preview','app_charge'])

    async def test_docking_waits_for_robot_video_shutdown(self):
        class ClosingRobot(Robot):
            closing=0
            async def send(self,duid,method,params):
                if method=='stop_camera_preview': self.closing=3
                if method=='app_charge': self.assert_closed=self.closing==0
                return await super().send(duid,method,params)
            async def snapshot(self,duid,position):
                st=await super().snapshot(duid,position)
                st['home_sec_status']=int(self.closing>0)
                self.closing=max(0,self.closing-1)
                return st
        r=ClosingRobot();p=self.runner(r)
        await p.start('robot',0,[{'kind':'home'}],'home')
        await self.finish(p)
        self.assertTrue(r.assert_closed)
        self.assertEqual(p.state['phase'],'complete')

    async def test_missing_browser_ack_never_docks(self):
        r=Robot();p=self.runner(r);p.CLOSE_TIMEOUT=.005
        await p.start('robot',0,[{'kind':'home'}],'home')
        await p.task
        self.assertEqual(p.state['phase'],'error')
        self.assertNotIn('app_charge',[m for m,_ in r.calls])
        self.assertIn('stop_camera_preview',[m for m,_ in r.calls])

    async def test_failed_remote_shutdown_never_docks(self):
        for failed in ('app_rc_end','stop_camera_preview'):
            class BrokenRobot(Robot):
                async def send(self,duid,method,params):
                    if method==failed: raise ValueError('shutdown refused')
                    return await super().send(duid,method,params)
            r=BrokenRobot();p=self.runner(r)
            await p.start('robot',0,[{'kind':'home'}],'home')
            await self.finish(p)
            self.assertEqual(p.state['phase'],'error')
            self.assertNotIn('app_charge',[m for m,_ in r.calls])

    async def test_remote_status_still_active_never_docks(self):
        class StuckCamera(Robot):
            async def snapshot(self,duid,position):
                return {**await super().snapshot(duid,position),'home_sec_status':1}
        r=StuckCamera();p=self.runner(r);p.UNLOCK_TIMEOUT=.005
        await p.start('robot',0,[{'kind':'home'}],'home')
        await self.finish(p)
        self.assertEqual(p.state['phase'],'error')
        self.assertNotIn('app_charge',[m for m,_ in r.calls])

    async def test_no_dock_coordinates_never_guesses_a_destination(self):
        r=Robot();r.dock_approach=None;p=self.runner(r)
        await p.start('robot',0,[{'kind':'home'}],'home')
        await self.finish(p)
        self.assertEqual(p.state['phase'],'error')
        self.assertEqual([m for m,_ in r.calls],['stop_camera_preview'])

    async def test_failed_approach_never_docks(self):
        r=Robot();r.arrive=False;p=self.runner(r)
        await p.start('robot',0,[{'kind':'home'}],'home')
        await self.finish(p)
        self.assertEqual(p.state['phase'],'error')
        self.assertEqual([m for m,_ in r.calls],['app_goto_target','app_stop','stop_camera_preview'])

    async def test_cancel_during_shutdown_never_docks(self):
        r=Robot();r.pos=r.dock_approach;p=self.runner(r)
        token=await p.start('robot',0,[{'kind':'home'}],'home')
        await asyncio.sleep(.005)
        await p.stop(token)
        self.assertEqual(p.state['phase'],'stopped')
        self.assertNotIn('app_charge',[m for m,_ in r.calls])

    async def test_already_docked_closes_without_moving(self):
        r=Robot();r.state=8;p=self.runner(r)
        await p.start('robot',0,[{'kind':'home'}],'home',False)
        await self.finish(p)
        self.assertEqual([m for m,_ in r.calls],['app_rc_end','stop_camera_preview'])
        self.assertTrue(p.state['end_session'])
        self.assertEqual(p.state['phase'],'complete')

    async def test_stop_during_travel_cannot_advance(self):
        r=Robot();r.arrive=False;p=self.runner(r)
        token=await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000},{'kind':'turn','degrees':90}],'route')
        await asyncio.sleep(.005)
        await p.stop(token)
        self.assertEqual(p.state['phase'],'stopped')
        self.assertEqual([c[0] for c in r.calls],['app_goto_target','app_stop','stop_camera_preview'])

    async def test_failed_arrival_cannot_rotate(self):
        r=Robot();r.arrive=False;p=self.runner(r)
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000},{'kind':'turn','degrees':90}],'route')
        await self.finish(p)
        self.assertEqual(p.state['phase'],'error')
        self.assertNotIn('app_rc_start',[c[0] for c in r.calls])
        self.assertIn('app_stop',[c[0] for c in r.calls])

    async def test_disconnect_stops_session(self):
        r=Robot();r.arrive=False;p=self.runner(r);p.LEASE=-1
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000}],'route')
        await self.finish(p)
        self.assertEqual(p.state['phase'],'error')
        self.assertIn('stop_camera_preview',[c[0] for c in r.calls])

    async def test_tap_arrival_keeps_camera(self):
        r=Robot();p=self.runner(r)
        await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000}],'tap',False)
        await self.finish(p)
        self.assertEqual(p.state['phase'],'complete')
        self.assertNotIn('stop_camera_preview',[c[0] for c in r.calls])

    async def test_stale_token_cannot_stop_or_renew(self):
        r=Robot();r.arrive=False;p=self.runner(r)
        token=await p.start('robot',0,[{'kind':'goto','x':2000,'y':1000}],'route')
        with self.assertRaises(ValueError):p.touch('old')
        with self.assertRaises(ValueError):await p.stop('old')
        await asyncio.sleep(.001)
        await p.stop(token)

    async def test_wrong_map_and_busy_robot_do_not_move(self):
        r=Robot();p=self.runner(r)
        with self.assertRaises(ValueError):await p.start('robot',1,[{'kind':'turn','degrees':90}],'route')
        r.state=5
        with self.assertRaises(ValueError):await p.start('robot',0,[{'kind':'turn','degrees':90}],'route')
        self.assertEqual(r.calls,[])

    async def test_stop_during_manual_pairing(self):
        r=Robot();p=self.runner(r);p.PAIR_DELAY=2
        token=await p.start('robot',0,[{'kind':'turn','degrees':90}],'route')
        await asyncio.sleep(.005);await p.stop(token)
        self.assertEqual([c[0] for c in r.calls],['app_rc_start','app_rc_end','stop_camera_preview'])


class Storage(unittest.TestCase):
    def test_save_reload_update_delete_and_device_isolation(self):
        with tempfile.TemporaryDirectory() as d:
            s=m.RouteStore(Path(d)/'routes.json')
            route=s.update('a',{'name':'Kitchen','steps':[{'kind':'goto','x':1234,'y':2345}]},0)
            self.assertEqual(m.RouteStore(s.path).list('a')[0]['steps'][0]['x'],1234)
            self.assertEqual(s.list('b'),[])
            with self.assertRaises(ValueError):s.update('b',{'id':route['id'],'delete':True},0)
            s.update('a',{'id':route['id'],'name':'Other','steps':[{'kind':'turn','degrees':-90,'speed':2}]},0)
            self.assertEqual(len(s.list('a')),1)
            self.assertEqual(m.RouteStore(s.path).list('a')[0]['steps'][0]['speed'],2)
            s.update('a',{'id':route['id'],'delete':True},0)
            self.assertEqual(s.list('a'),[])

    def test_invalid_steps(self):
        for steps in ([],[{'kind':'goto','x':float('nan'),'y':0}],[{'kind':'goto','x':60000,'y':0}],
                      [{'kind':'home'},{'kind':'wait','seconds':1}],[{'kind':'turn','degrees':10000}],[{'kind':'wait','seconds':-1}],[{'kind':'oops'}]):
            with self.assertRaises(ValueError):m.validate_steps(steps)

    def test_rotation_speed_validation_and_legacy_default(self):
        self.assertEqual(m.validate_steps([{'kind':'turn','degrees':90}])[0]['speed'],1)
        for speed in (None, True, 'fast', 0, -1, 3, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                m.validate_steps([{'kind':'turn','degrees':90,'speed':speed}])



class DockGeometry(unittest.TestCase):
    def test_cardinal_directions_and_rotated_flipped_projection(self):
        for angle,delta in [(0,[800,0]),(90,[0,800]),(180,[-800,0]),(-90,[0,-800])]:
            dock=SimpleNamespace(x=25000,y=26000,a=angle)
            proj=(lambda x,y:(x/512,y/512),512,512)
            result=m.station_approach(dock,proj,lambda x,y:(x*100,y*100))
            self.assertEqual(result,[25000+delta[0],26000+delta[1]])
            # Rotate the rendered image and invert its Y axis; inverse calibration
            # still yields the same native point.
            proj=(lambda x,y:((51200-y)/512,x/512),512,512)
            result=m.station_approach(dock,proj,lambda x,y:(y*100,51200-x*100))
            self.assertEqual(result,[25000+delta[0],26000+delta[1]])
    def test_missing_or_invalid_dock_is_rejected(self):
        proj=(lambda x,y:(x,y),100,100)
        for dock in [None,SimpleNamespace(x=0,y=0,a=None),SimpleNamespace(x=0,y=0,a=float('nan')),SimpleNamespace(x=0,y=0,a=180)]:
            self.assertIsNone(m.station_approach(dock,proj,lambda x,y:(x,y)))

if __name__=='__main__':unittest.main()
