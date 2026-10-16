#!/usr/bin/env python3
"""Regenerate the editable V550 indoor world using only local SDF primitives."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def make_world():
    root = ET.Element('sdf', version='1.6')
    world = ET.SubElement(root, 'world', name='v550_rooms')

    def element(parent, tag, value):
        node = ET.SubElement(parent, tag)
        node.text = str(value)
        return node

    element(world, 'gravity', '0 0 -9.8')
    physics = ET.SubElement(world, 'physics', type='ode')
    element(physics, 'max_step_size', 0.001)
    element(physics, 'real_time_update_rate', 1000)
    scene = ET.SubElement(world, 'scene')
    element(scene, 'ambient', '0.65 0.65 0.65 1')
    element(scene, 'background', '0.70 0.73 0.77 1')
    element(scene, 'shadows', 'false')
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    element(light, 'pose', '0 0 15 0 0 0')
    element(light, 'diffuse', '0.8 0.8 0.8 1')
    element(light, 'direction', '-0.3 0.2 -1')
    element(light, 'cast_shadows', 'false')
    camera = ET.SubElement(ET.SubElement(world, 'gui'), 'camera', name='user_camera')
    element(camera, 'pose', f'0 0 36 0 {math.pi/2} {math.pi/2}')
    element(camera, 'view_controller', 'orbit')

    def solid(name, x, y, size, color, yaw=0.0, cylinder=False, z=None, collision=True):
        height = size[-1]
        model = ET.SubElement(world, 'model', name=name)
        element(model, 'static', 'true')
        element(model, 'pose', f'{x} {y} {height/2 if z is None else z} 0 0 {yaw}')
        link = ET.SubElement(model, 'link', name='link')
        for kind in (['collision', 'visual'] if collision else ['visual']):
            node = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(node, 'geometry')
            shape = ET.SubElement(geometry, 'cylinder' if cylinder else 'box')
            if cylinder:
                element(shape, 'radius', size[0]); element(shape, 'length', height)
            else:
                element(shape, 'size', ' '.join(map(str, size)))
            if kind == 'visual':
                material = ET.SubElement(node, 'material')
                element(material, 'ambient', color)
                element(material, 'diffuse', color)

    white, brown = '0.83 0.82 0.78 1', '0.36 0.18 0.07 1'
    blue, grey = '0.20 0.38 0.53 1', '0.40 0.45 0.48 1'
    solid('floor', 0, 0, (24.4, 18.4, 0.1), '0.55 0.55 0.55 1', z=-0.05)
    # A visual strip makes the main corridor legible without creating a laser return.
    solid('corridor_floor_marking', 0, 0, (23.8, 3.8, 0.002), '0.66 0.64 0.57 1',
          z=0.001, collision=False)
    for y, name in [(9, 'north'), (-9, 'south')]:
        solid('outer_'+name, 0, y, (24.2, 0.18, 1.5), white)
    for x, name in [(-12, 'west'), (12, 'east')]:
        solid('outer_'+name, x, 0, (0.18, 18.2, 1.5), white)
    # Six rooms open into a 4 m wide corridor. Clear doorway width: 1.9 m.
    for sign, side in [(1, 'north'), (-1, 'south')]:
        for index, (left, right) in enumerate([(-12, -8.95), (-7.05, -0.95),
                                                (0.95, 7.05), (8.95, 12)]):
            solid(f'{side}_front_{index}', (left+right)/2, sign*2,
                  (right-left, 0.16, 1.5), white)
        for x in [-4, 4]:
            solid(f'{side}_partition_{x}', x, sign*5.5, (0.16, 7, 1.5), white)
        for i, x in enumerate([-8, 0, 8]):
            # Open fixed door leaf, jambs and a lintel; no hidden collision across the opening.
            solid(f'{side}_door_{i}_open_leaf', x-0.90, sign*2.88,
                  (0.055, 1.70, 1.35), brown)
            for j, offset in enumerate([-0.95, 0.95]):
                solid(f'{side}_door_{i}_jamb_{j}', x+offset, sign*2,
                      (0.08, 0.24, 1.60), brown)
            solid(f'{side}_door_{i}_lintel', x, sign*2,
                  (1.98, 0.24, 0.12), brown, z=1.60)
            # Four obstacles per room, away from the central door approach.
            solid(f'{side}_room_{i}_cabinet', x+2.8, sign*6.4, (0.65, 2.4, 1.25), blue)
            solid(f'{side}_room_{i}_desk', x-0.6, sign*6.9, (1.8, 0.85, 0.75), brown)
            solid(f'{side}_room_{i}_crate', x+1.7, sign*4.6, (0.70, 0.70, 0.65), grey,
                  yaw=sign*0.25)
            solid(f'{side}_room_{i}_barrel', x-2.5, sign*5.0, (0.35, 0.9), blue,
                  cylinder=True)
    # Six staggered corridor obstacles; keep the six door approaches open.
    for i, (x, y, radius) in enumerate([(-6, 0.3, 0.35), (2.5, -0.45, 0.32),
                                       (-10.6, 1.1, 0.28), (-3.8, -1.1, 0.28),
                                       (5.4, 1.1, 0.28), (10.3, -1.1, 0.28)]):
        solid(f'corridor_barrel_{i}', x, y, (radius, 0.85), blue, cylinder=True)
    ET.indent(root, space='  ')
    return ET.ElementTree(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    make_world().write(args.output, encoding='utf-8', xml_declaration=True)
    print(args.output)


if __name__ == '__main__':
    main()
