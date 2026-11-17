#!/usr/bin/env python3
"""Validate both preview and simulated model against the original URDF."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro


root = Path(__file__).resolve().parents[1]
description = root / 'src/robot_description'
source = description / 'xacro/baobaowan_B_800mm.urdf.xacro'
original = ET.parse(description / 'urdf/baobaowan_B_800mm.urdf').getroot()
preview = ET.fromstring(xacro.process_file(str(source)).toxml())
simulation = ET.fromstring(xacro.process_file(str(source), mappings={
    'gazebo': 'true', 'joint_damping': '0.001', 'lidar_downsample': '5'}).toxml())
assert {l.get('name') for l in preview.findall('link')} == {l.get('name') for l in original.findall('link')}
for joint in original.findall('joint'):
    peer = preview.find(f"joint[@name='{joint.get('name')}']")
    assert peer is not None and joint.get('type') == peer.get('type')
    for key in ['xyz', 'rpy']:
        assert np.allclose([float(x) for x in joint.find('origin').get(key).split()],
                           [float(x) for x in peer.find('origin').get(key).split()], atol=1e-10)
for tree in [preview, simulation]:
    links = {l.get('name') for l in tree.findall('link')}
    parents = {}
    for joint in tree.findall('joint'):
        a, b = joint.find('parent').get('link'), joint.find('child').get('link')
        assert a in links and b in links and b not in parents
        parents[b] = a
    assert links - parents.keys() == {'base_footprint'}
    for link in tree.findall('link'):
        inertia = link.find('inertial/inertia')
        if inertia is not None:
            x, y, z, xy, xz, yz = [float(inertia.get(k)) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']]
            assert np.linalg.eigvalsh([[x, xy, xz], [xy, y, yz], [xz, yz, z]]).min() > 0
        for mesh in link.findall('.//mesh'):
            name = mesh.get('filename')
            if name.startswith('package://robot_description/'):
                assert (description / name.removeprefix('package://robot_description/')).is_file(), name
mount = simulation.find("joint[@name='mid360_mount_joint']/origin")
assert np.allclose([float(x) for x in mount.get('rpy').split()], [0, np.pi/6, 0])
assert preview.find("link[@name='imu_link']") is None
assert simulation.find("link[@name='imu_link']") is not None
assert simulation.find("link[@name='scan_link']") is not None
for name in ['left_wheel_link', 'right_wheel_link', 'front_caster_wheel_link', 'rear_caster_wheel_link']:
    assert simulation.find(f"link[@name='{name}']/collision/geometry/cylinder") is not None
report = {'passed': True, 'preview_links': len(preview.findall('link')),
          'simulation_links': len(simulation.findall('link')),
          'simulation_joints': len(simulation.findall('joint')),
          'sensors': [(s.get('name'), s.get('type')) for s in simulation.findall('.//sensor')],
          'preview_joint_poses_match_original': True, 'positive_inertias': True,
          'all_robot_meshes_found': True}
text = json.dumps(report, ensure_ascii=False, indent=2)
print(text)
(root / 'results/model_validation.json').write_text(text + '\n')
