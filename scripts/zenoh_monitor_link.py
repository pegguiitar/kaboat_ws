#!/usr/bin/env python3
"""Run a narrowly scoped ROS 2 DDS bridge over the two hosts' Tailscale link.

Jetson ROS domain 42 -> laptop ROS domain 43 carries monitoring topics.
The only laptop -> Jetson route is /boat_pose for the indoor shore LiDAR.
"""

import argparse
import ipaddress
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


TAILSCALE_RANGE = ipaddress.ip_network('100.64.0.0/10')
PORT = 7447

# These are exact ROS interface names. Images and motor commands are excluded.
MONITOR_TOPICS = (
    '/scan',
    '/scan/raw',
    '/odom',
    '/tf',
    '/tf_static',
    '/occupancy_grid',
    '/avoid/plan',
    '/avoid/cps',
    '/avoid/dri',
    '/real_avoid_test/path',
    '/real_avoid_test/markers',
    '/real_avoid_test/dri',
    '/real_avoid_test/status',
    '/straight_drive/markers',
    '/bspline_test/path',
    '/bspline_test/markers',
    '/circle_drive/markers',
    '/station_keeping/markers',
    '/dock/waypoints',
    '/dock/entrance_base',
    '/dock/marker_base',
    '/dock/detection_status',
    '/mission/state',
    '/mission/goal',
    '/imu/data',
    '/gps/fix',
    '/gnss_1/velocity',
    '/ekf/status',
    '/ekf/odometry_map',
)


def tailscale_ip(value):
    """Require a Tailscale IPv4 address, not a changing hotspot/LAN address."""
    try:
        address = ipaddress.ip_address(value.strip())
    except ValueError as exc:
        raise ValueError(f'invalid IP address: {value!r}') from exc
    if address not in TAILSCALE_RANGE:
        raise ValueError(f'{address} is not in the Tailscale 100.64.0.0/10 range')
    return str(address)


def get_local_tailscale_ip():
    if not shutil.which('tailscale'):
        raise RuntimeError('tailscale is not installed on this host')
    status = subprocess.run(
        ['tailscale', 'status', '--json'], capture_output=True, text=True,
        check=True)
    if json.loads(status.stdout).get('BackendState') != 'Running':
        raise RuntimeError('Tailscale is not connected; run sudo tailscale up')
    result = subprocess.run(
        ['tailscale', 'ip', '-4'], capture_output=True, text=True, check=True)
    return tailscale_ip(result.stdout)


def build_config(role, local_ip, jetson_ip=None, domain=None):
    if role not in ('jetson', 'laptop'):
        raise ValueError('role must be jetson or laptop')
    local_ip = tailscale_ip(local_ip)
    if role == 'laptop':
        if jetson_ip is None:
            raise ValueError('laptop requires --jetson-ip')
        jetson_ip = tailscale_ip(jetson_ip)
        if local_ip == jetson_ip:
            raise ValueError('laptop and Jetson Tailscale IPs must differ')
    domain = (42 if role == 'jetson' else 43) if domain is None else domain
    if not 0 <= domain <= 101:
        raise ValueError('ROS_DOMAIN_ID must be between 0 and 101')

    def exact(name):
        # Config patterns are regular expressions matching the full ROS name.
        return '^' + name + '$'

    allowed = {
        'publishers': [exact(topic) for topic in (
            MONITOR_TOPICS if role == 'jetson' else ('/boat_pose',))],
        'subscribers': [exact(topic) for topic in (
            ('/boat_pose',) if role == 'jetson' else MONITOR_TOPICS)],
        'service_servers': [],
        'service_clients': [],
        'action_servers': [],
        'action_clients': [],
    }
    config = {
        'mode': 'router',
        'plugins': {'ros2dds': {'domain': domain, 'allow': allowed}},
    }
    if role == 'jetson':
        # No listener on the phone hotspot or other LAN interfaces.
        config['listen'] = {'endpoints': [f'tcp/{local_ip}:{PORT}']}
    else:
        config['listen'] = {'endpoints': ['tcp/127.0.0.1:0']}
        config['connect'] = {
            'endpoints': [f'tcp/{jetson_ip}:{PORT}'],
            'exit_on_failure': False,
            'retry': {
                'period_init_ms': 1000,
                'period_max_ms': 4000,
                'period_increase_factor': 2,
            },
        }
    return config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=('jetson', 'laptop'))
    parser.add_argument('--jetson-ip', help='Jetson Tailscale IPv4; laptop only')
    parser.add_argument('--domain', type=int,
                        help='local ROS_DOMAIN_ID (defaults: Jetson 42, laptop 43)')
    parser.add_argument('--print-config', action='store_true',
                        help='print generated config and exit without starting Zenoh')
    args = parser.parse_args(argv)
    if args.role == 'jetson' and args.jetson_ip:
        parser.error('--jetson-ip is only used on the laptop')
    if args.role == 'laptop' and not args.jetson_ip:
        parser.error('laptop requires --jetson-ip from `tailscale ip -4` on Jetson')

    try:
        local_ip = get_local_tailscale_ip()
        config = build_config(args.role, local_ip, args.jetson_ip, args.domain)
        domain = config['plugins']['ros2dds']['domain']
        current_domain = os.environ.get('ROS_DOMAIN_ID')
        if current_domain not in (None, str(domain)):
            raise RuntimeError(
                f'ROS_DOMAIN_ID={current_domain} differs from {role_label(args.role)} '
                f'domain {domain}; check the local ROS nodes before starting')
        if args.print_config:
            print(json.dumps(config, indent=2))
            return 0
        binary = shutil.which('zenoh-bridge-ros2dds')
        if not binary:
            raise RuntimeError('zenoh-bridge-ros2dds is not installed on this host')

        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        if args.role == 'jetson':
            print(f'Jetson: listening on Tailscale {local_ip}:{PORT}; ROS domain {domain}',
                  flush=True)
        else:
            print(f'Laptop: connecting to Jetson {args.jetson_ip}:{PORT}; '
                  f'ROS domain {domain}', flush=True)
        with tempfile.TemporaryDirectory(prefix='kaboat-zenoh-') as tmp:
            config_path = Path(tmp) / 'bridge.json5'
            config_path.write_text(json.dumps(config, indent=2) + '\n')
            return subprocess.call([binary, '-c', str(config_path)], env=env)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError,
            json.JSONDecodeError) as exc:
        print(f'zenoh monitor link: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


def role_label(role):
    return 'Jetson' if role == 'jetson' else 'laptop'


if __name__ == '__main__':
    sys.exit(main())
