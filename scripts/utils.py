from path_plans import InclinedPlane
from path_plans import SteppedRings
from path_plans import OrthogonalCapture
import numpy as np
import argparse
import inspect
import json
import sys
import os
current = os.path.dirname(os.path.realpath(__file__))


def get_path_from_json(structure):
    """ Accepts a dictionary from a JSON file with information about an object
    and settings for a path plan and returns the specified path plan.

    Assumes any parameters set to None (null in JSON notation) should be
    the default value.

    @param structure: dictionary of of a loaded json file.
    """
    size = structure['ObjectInfo']['size']
    position = structure['ObjectInfo']['position']
    orientation = np.array(structure['ObjectInfo']['orientation'])
    path_type = structure['PathType']
    if path_type == 'SteppedRings':
        a = inspect.getargspec(SteppedRings.__init__)
        defaults = dict(zip(a.args[-len(a.defaults):],a.defaults))

        opt = structure['SteppedRings']
        for k in opt.keys():
            if opt[k] is None:
                opt[k] = defaults[k]

        path = SteppedRings(size,
                            position,
                            orientation,
                            scale=opt['scale'],
                            offset=opt['offset'],
                            level_count=opt['level_count'],
                            density=opt['density'],
                            )
    elif path_type == 'InclinedPlane':
        a = inspect.getargspec(InclinedPlane.__init__)
        defaults = dict(zip(a.args[-len(a.defaults):],a.defaults))

        opt = structure['InclinedPlane']
        for k in opt.keys():
            if opt[k] is None:
                opt[k] = defaults[k]

        path = InclinedPlane(size,
                             position,
                             orientation,
                             count=opt['count'],
                             clearance=opt['clearance'],
                             plane_scale=opt['plane_scale'],
                             slope=opt['slope'],
                             offset=opt['offset'],
                             )
    else:
        path = None
        exit('[ERROR] Invalid path type in JSON file.')

    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Input JSON file name")
    parser.add_argument('-json_name', '--json-name', type=str, default='detected_object.json')
    args = parser.parse_args()
    json_name = args.json_name
    fname = os.path.join(current, json_name)
    print str(json_name)
    with open(fname, "r") as read_file:
        json_structure = json.load(read_file)

    # get_path_from_json(json_structure)