#!/usr/bin/env python3
"""Save the running Gazebo scene, removing the V550 that the launch will spawn."""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from google.protobuf import text_format
from google.protobuf.wrappers_pb2 import StringValue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='Destination .world file')
    parser.add_argument('--world-name', help='Optional Gazebo world name')
    parser.add_argument('--exclude-model', action='append', default=[],
                        help='Additional model to exclude (repeatable)')
    args = parser.parse_args()
    command = ['gz', 'topic']
    if args.world_name:
        command += ['-w', args.world_name]
    command += ['-r', 'world_sdf']
    response = subprocess.check_output(command, text=True, timeout=15)
    # gz formats gazebo.msgs.GzString as protobuf text, with a single data field.
    # StringValue has the same string representation under the field "value".
    message = text_format.Parse(response.replace('data:', 'value:', 1), StringValue())
    root = ET.fromstring(message.value)
    world = root.find('world')
    if world is None:
        raise RuntimeError('Gazebo returned no SDF world')
    excluded = {'v550_mec', *args.exclude_model}
    for parent in [world, *world.findall('state')]:
        for model in list(parent.findall('model')):
            if model.get('name') in excluded:
                parent.remove(model)
    target = Path(args.output).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    print(f'Saved {len(world.findall("model"))} scene models to {target}; V550 will spawn at launch.')


if __name__ == '__main__':
    main()
