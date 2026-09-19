#!/usr/bin/env python3
"""Compute the UDP ports DDS (RTPS) uses for a ROS_DOMAIN_ID (lesson 04.01). No ROS needed.

Constants are the defaults of the OMG DDS-RTPS specification, used by Fast DDS,
Cyclone DDS and Connext out of the box.

  python3 dds_ports.py 0            # domain 0, participants 0..2
  python3 dds_ports.py 42 --participants 5
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass

PB = 7400   # port base
DG = 250    # domain id gain
PG = 2      # participant id gain
D0, D1, D2, D3 = 0, 10, 1, 11   # offsets
MAX_PORT = 65535


@dataclass(frozen=True)
class DomainPorts:
    domain_id: int
    discovery_multicast: int     # SPDP announcements, multicast group 239.255.0.1
    user_multicast: int          # data sent to multicast (rare in ROS 2)

    def discovery_unicast(self, participant_id: int) -> int:
        return PB + DG * self.domain_id + D1 + PG * participant_id

    def user_unicast(self, participant_id: int) -> int:
        return PB + DG * self.domain_id + D3 + PG * participant_id


def ports_for(domain_id: int) -> DomainPorts:
    if not 0 <= domain_id <= 232:
        raise ValueError('ROS_DOMAIN_ID must be 0..232 (0..101 recommended on Linux)')
    base = PB + DG * domain_id
    return DomainPorts(domain_id, base + D0, base + D2)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('domain_id', type=int)
    ap.add_argument('--participants', type=int, default=3,
                    help='how many ROS processes (DDS participants) to list')
    args = ap.parse_args()
    p = ports_for(args.domain_id)
    print(f'ROS_DOMAIN_ID={p.domain_id}')
    print(f'  discovery multicast : udp/{p.discovery_multicast}')
    print(f'  user multicast      : udp/{p.user_multicast}')
    for i in range(args.participants):
        print(f'  participant {i:3d}     : discovery udp/{p.discovery_unicast(i)}, '
              f'data udp/{p.user_unicast(i)}')
    last = p.user_unicast(args.participants - 1)
    if last > MAX_PORT:
        print(f'  !! participant {args.participants - 1} would need port {last} > {MAX_PORT}')


if __name__ == '__main__':
    main()
