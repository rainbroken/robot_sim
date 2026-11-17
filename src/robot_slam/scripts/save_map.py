#!/usr/bin/env python3
import argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SavePcdMap


def main():
    parser = argparse.ArgumentParser(description='Save loop-optimized 3D and occupancy maps')
    parser.add_argument('directory')
    args = parser.parse_args()
    directory = Path(args.directory).expanduser().resolve()
    if directory.exists() and any(directory.iterdir()):
        parser.error('Use a new or empty directory to avoid overwriting another map')
    directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('save_slam_map')
    client = node.create_client(SavePcdMap, '/mapping/save_pcd_map')
    if not client.wait_for_service(timeout_sec=15):
        raise RuntimeError('Mapping save service is not available')
    request = SavePcdMap.Request(output_directory=str(directory))
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=180)
    if not future.done():
        raise RuntimeError('Save request timed out; inspect the output before retrying')
    result = future.result()
    print(result)
    if not result.success or result.occupancy_empty:
        raise RuntimeError('Map save failed or occupancy is empty; collect more coverage')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
