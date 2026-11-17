#!/usr/bin/env python3
"""Bounded HTTP -> ROS -> Gazebo workflow test. Requires an idle Web + house sim.

Uses only the Web control API, does not call ROS CLI to map or navigate.
The short corridor route assumes the shipped house spawn; not a generic explorer.
"""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.error
import urllib.request
import uuid


class Client:
    def __init__(self, url):
        self.url = url.rstrip('/')
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.owner, self.sequence = uuid.uuid4().hex, 0

    def get(self, route):
        with self.opener.open(self.url + '/api/' + route, timeout=8) as response:
            return json.load(response)

    def post(self, action, data=None):
        request = urllib.request.Request(self.url + '/api/command',
            data=json.dumps({'action': action, 'data': data or {}}).encode(),
            headers={'Content-Type': 'application/json'})
        try:
            with self.opener.open(request, timeout=8) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(exc.read().decode()) from exc

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        state = {}
        while time.monotonic() < deadline:
            try:
                state = self.get('state')
            except (urllib.error.URLError, OSError):
                time.sleep(.2)
                continue
            if predicate(state):
                return state
            time.sleep(.2)
        raise RuntimeError(f'Timeout waiting for {description}: {state}')

    def job(self, action, data=None, timeout=45):
        job = self.post(action, data)
        result = self.wait(lambda s: s.get('job', {}).get('id') == job['id'] and
                           s['job']['state'] != 'running', timeout, action)['job']
        if result['state'] != 'succeeded':
            raise RuntimeError(result['message'])
        return result

    def velocity(self, linear=0., angular=0.):
        self.sequence += 1
        return self.post('velocity', {'client_id': self.owner, 'sequence': self.sequence,
                                     'linear': linear, 'angular': angular})

    def release(self):
        self.sequence += 1
        return self.post('release', {'client_id': self.owner, 'sequence': self.sequence})

    def drive(self, speed, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.velocity(speed)
            time.sleep(.1)
        self.release()
        time.sleep(.7)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8080')
    parser.add_argument('--map-name', default='web_test_' + time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--output', required=True)
    parser.add_argument('--reuse-mapping', action='store_true', help='Use already running Web-owned mapping')
    parser.add_argument('--goal-offset', type=float, default=.55,
                        help='Goal offset along map X; use -.55 to probe the harder rear-goal turn')
    args = parser.parse_args()
    client = Client(args.url)
    report = {'map_name': args.map_name, 'goal_offset': args.goal_offset,
              'started_at': time.strftime('%Y-%m-%dT%H:%M:%S'), 'checks': {}}
    try:
        initial = client.wait(lambda s: s.get('sim_ready'), 60, 'Web and Gazebo startup')
        if not args.reuse_mapping and initial['backend']['mode'] != 'idle':
            raise RuntimeError('Start with idle Web; this test does not take over active backends')
        if args.reuse_mapping and initial['backend']['mode'] != 'mapping':
            raise RuntimeError('--reuse-mapping requires an existing Web mapping backend')
        if not args.reuse_mapping:
            client.job('start_mapping')
        ready = client.wait(lambda s: s['mapping_ready'] and s['manual_ready'], 60, 'mapping initialization')
        report['checks']['mapping_ready'] = True
        start_pose = ready['pose']
        # Move slightly in the known spawn corridor and deliberately stop renewals.
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            client.velocity(.22)
            time.sleep(.1)
        moved = client.get('state')
        time.sleep(1.2)  # No release packet: exercise the server-side deadman.
        stopped = client.get('state')
        report['checks']['deadman'] = {'before_speed': moved['odom']['linear'],
            'after_speed': stopped['odom']['linear'], 'stopped': abs(stopped['odom']['linear']) < .03}
        if not report['checks']['deadman']['stopped']:
            raise RuntimeError('Deadman did not stop the robot')
        client.drive(.22, 5)
        client.drive(-.15, 7)
        client.drive(.22, 2)
        final_mapping = client.get('state')
        report['checks']['teleop_displacement'] = math.dist(start_pose[:2], final_mapping['pose'][:2])
        client.job('save_map', {'name': args.map_name}, timeout=190)
        maps = client.get('maps')['maps']
        if not any(m['name'] == args.map_name and m['ready'] for m in maps):
            raise RuntimeError('Saved map is missing from ready list')
        report['checks']['saved_map'] = True
        client.job('start_navigation', {'name': args.map_name}, timeout=50)
        # Same simulation is retained, so set the pose observed at the end of mapping.
        pose = final_mapping['pose']
        client.wait(lambda s: s['backend']['mode'] == 'navigation', 10, 'navigation process')
        client.wait(lambda s: s.get('frontend_ready'), 60, 'localization frontend initialization')
        client.job('initial_pose', dict(zip(('x', 'y', 'yaw'), pose)), timeout=20)
        ready = client.wait(lambda s: s['navigation_ready'], 90, 'TRACKING + Nav2 active')
        report['checks']['navigation_ready'] = {'localization': ready['localization'], 'pose': ready['pose']}
        grid = client.get('map')
        report['checks']['occupancy'] = {'width': grid['info']['width'], 'height': grid['info']['height'],
                                         'known_cells': sum(v >= 0 for v in grid['data'])}
        # Default is a short forward goal in the surveyed corridor. A rear goal
        # also tests large turns and depends on the 3D map's viewpoint coverage.
        current = ready['pose']
        goal = {'x': current[0] + args.goal_offset, 'y': current[1], 'yaw': 0.0}
        client.job('goal', goal)
        terminal = client.wait(lambda s: s['navigation']['state'] in ('succeeded', 'aborted', 'canceled', 'error'),
                               100, 'navigation result')
        error = math.hypot(terminal['pose'][0]-goal['x'], terminal['pose'][1]-goal['y'])
        report['checks']['navigation_goal'] = {'goal': goal, 'result': terminal['navigation'],
                                             'final_pose': terminal['pose'], 'xy_error': error}
        if terminal['navigation']['state'] != 'succeeded':
            raise RuntimeError('Navigation did not succeed')
        # Confirm cancellation is a Nav2 action result, not just a zero Twist.
        p = terminal['pose']
        cancel_goal = {'x': p[0]+.55, 'y': p[1], 'yaw': 0.0}
        client.job('goal', cancel_goal)
        goal_values = [cancel_goal[k] for k in ('x', 'y', 'yaw')]
        client.wait(lambda s: s['navigation'].get('goal') == goal_values and
                    s['navigation']['state'] == 'navigating', 5, 'new goal feedback')
        client.post('stop')
        canceled = client.wait(lambda s: s['navigation'].get('goal') == goal_values and
                               s['navigation']['state'] == 'canceled', 12, 'CANCELED result of new goal')
        report['checks']['cancel'] = canceled['navigation']
        client.job('stop_backend')
        report['checks']['backend_stop'] = client.get('state')['backend']
        report['success'] = True
    except Exception as exc:
        report['success'], report['error'] = False, str(exc)
        try:
            client.post('stop')
            report['last_state'] = client.get('state')
            report['last_state']['scan'] = len(report['last_state'].get('scan', []))
            report['log_tail'] = client.get('log')['text']
        except Exception:
            pass
    finally:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report.get('success'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
