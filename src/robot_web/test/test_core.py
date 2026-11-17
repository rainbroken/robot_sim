import math
import subprocess
import sys
from types import SimpleNamespace as S

import pytest

from robot_web.core import Backend, MapStore, Occupancy, VelocityLease, finite


def complete_map(root, name='house'):
    directory = root / name
    directory.mkdir()
    for filename in ('localization.pcd', 'full_map.pcd'):
        (directory / filename).write_text('VERSION .7\nPOINTS 1\nDATA ascii\n0 0 0\n')
    (directory / 'occupancy.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\x00\x00')
    (directory / 'occupancy.yaml').write_text('image: occupancy.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    return directory


def test_map_directory_traversal_and_partial_saves(tmp_path):
    maps = MapStore(tmp_path)
    for name in ('../elsewhere', '/tmp/map', '', '.', 'a/b', 'a b', 'x'*65):
        with pytest.raises(ValueError):
            maps.directory(name)
    complete_map(tmp_path)
    assert maps.validate('house') == tmp_path / 'house'
    with pytest.raises(FileExistsError):
        maps.reserve('house')
    maps.reserve('partial')
    assert maps.list() == [{'name': 'house', 'ready': True, 'error': ''},
                           {'name': 'partial', 'ready': False, 'error': '地图不完整或路径不合法：localization.pcd'}]


def test_map_symlinks_cannot_escape(tmp_path):
    root = tmp_path / 'maps'
    maps = MapStore(root)
    outside = complete_map(tmp_path, 'outside')
    (root / 'escape').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError):
        maps.validate('escape')
    directory = complete_map(root)
    (directory / 'localization.pcd').unlink()
    (directory / 'localization.pcd').symlink_to(outside / 'localization.pcd')
    with pytest.raises(ValueError):
        maps.validate('house')


def test_goal_checks_rotated_map_and_incremental_bounds():
    grid = Occupancy()
    origin = S(position=S(x=10., y=20.), orientation=S(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4)))
    grid.full(S(info=S(width=2, height=2, resolution=1., origin=origin),
                header=S(frame_id='map'), data=[0, -1, 100, 0]))
    assert grid.contains_free(9.5, 20.5)
    assert not grid.contains_free(9.5, 21.5)  # Unknown cell after map rotation.
    assert not grid.contains_free(8.5, 20.5)  # Obstacle.
    assert not grid.contains_free(20., 20.)
    rev = grid.revision
    grid.update(S(x=0, y=1, width=1, height=1, header=S(frame_id='map'), data=[0]))
    assert grid.revision == rev + 1
    assert grid.contains_free(8.5, 20.5)
    grid.update(S(x=2, y=1, width=1, height=1, header=S(frame_id='map'), data=[100]))
    assert len(grid.data) == 4 and grid.revision == rev + 1
    grid.reset()
    assert grid.revision == rev + 2 and not grid.contains_free(9.5,20.5)


def test_velocity_bounds_ownership_and_deadman():
    lease = VelocityLease()
    lease.set('browser-1', .2, -.3, 100.)
    assert lease.until == 100.35
    with pytest.raises(ValueError):
        lease.set('browser-2', .2, 0., 100.1)
    lease.set('browser-2', .1, 0., 100.4)
    assert lease.owner == 'browser-2'
    for value in (float('nan'), float('inf'), -.21, .31, True):
        with pytest.raises(ValueError):
            finite(value, -.2, .3)
    lease.clear()
    assert lease.command == (0., 0.) and lease.until == 0.


def test_owned_process_cleanup_does_not_kill_unrelated_process(tmp_path):
    backend = Backend(tmp_path)
    owned = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], start_new_session=True)
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], start_new_session=True)
    try:
        backend.process = owned
        backend.mode = 'mapping'
        backend.stop()
        assert owned.poll() is not None
        assert unrelated.poll() is None
        assert backend.snapshot()['mode'] == 'idle'
    finally:
        if owned.poll() is None:
            owned.kill()
        unrelated.terminate()
        unrelated.wait(timeout=3)
