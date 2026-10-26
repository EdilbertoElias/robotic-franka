"""Write a printable SVG (exact size in mm) of the workbench ArUco marker.

ros2 run aided_building_hand_perception generate_marker -- --output marker.svg
"""
import argparse

from aided_building_hand_perception.aruco import draw_marker, get_dictionary


def marker_svg(dictionary_name, marker_id, size_mm):
    dictionary = get_dictionary(dictionary_name)
    # One pixel per cell: 4x4 data bits plus a one-cell black border
    cells = dictionary.markerSize + 2
    bits = draw_marker(dictionary, marker_id, cells)
    cell_mm = size_mm / cells
    # The detector needs a white quiet zone of at least one cell around the marker
    margin_mm = 2.0 * cell_mm

    width = size_mm + 2 * margin_mm
    height = size_mm + 2 * margin_mm + 25.0
    rects = []
    for row in range(cells):
        for col in range(cells):
            if bits[row, col] == 0:
                rects.append(
                    f'<rect x="{margin_mm + col * cell_mm:.3f}" y="{margin_mm + row * cell_mm:.3f}" '
                    f'width="{cell_mm:.3f}" height="{cell_mm:.3f}"/>')

    bar_y = size_mm + 2 * margin_mm
    label = (f'{dictionary_name} id={marker_id} lado={size_mm:g} mm. '
             f'Imprima em 100% (sem ajustar a pagina) e confira a barra de 50 mm.')
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}mm" height="{height}mm" viewBox="0 0 {width} {height}">
  <rect width="{width}" height="{height}" fill="white"/>
  <g fill="black" shape-rendering="crispEdges">
    {''.join(rects)}
  </g>
  <line x1="{margin_mm}" y1="{margin_mm + size_mm / 2}" x2="{margin_mm - 5}" y2="{margin_mm + size_mm / 2}" stroke="red" stroke-width="0.4"/>
  <text x="{margin_mm + size_mm + 2}" y="{margin_mm + size_mm / 2 + 1.5}" font-size="4" fill="red">+x</text>
  <text x="{margin_mm + size_mm / 2 - 2}" y="{margin_mm - 3}" font-size="4" fill="red">+y</text>
  <rect x="{margin_mm}" y="{bar_y}" width="50" height="3" fill="black"/>
  <text x="{margin_mm}" y="{bar_y + 9}" font-size="3.5">50 mm</text>
  <text x="{margin_mm}" y="{bar_y + 16}" font-size="3">{label}</text>
</svg>
"""


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--id', type=int, default=0)
    parser.add_argument('--size-mm', type=float, default=100.0)
    parser.add_argument('--output', default='workbench_marker.svg')
    options = parser.parse_args(args)

    with open(options.output, 'w') as svg:
        svg.write(marker_svg(options.dictionary, options.id, options.size_mm))
    print(f'Wrote {options.output} ({options.dictionary} id={options.id}, '
          f'{options.size_mm:g} mm)')


if __name__ == '__main__':
    main()
