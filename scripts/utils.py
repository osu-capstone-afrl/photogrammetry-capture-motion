from path_plans import InclinedPlane
from path_plans import SteppedRings
from path_plans import OrthogonalCapture
import numpy as np
import json
import sys
import os
current = os.path.dirname(os.path.realpath(__file__))


def get_path_from_json(structure):
    """ Accepts a dictionary from a JSON file with information about an object
    and settings for a path plan and returns the specified path plan.

    Assumes any parameters set to None (null in JSON notation) should be
    the default value.

    **Note that the function is inefficient**, creating two path plans. This is to get
    the default parameter values from path_plans.py. There's likely a better way to do this
    and should be updates. But this works for now

    @param structure: dictionary of of a loaded json file.
    """
    size = structure['size']
    position = structure['position']
    orientation = np.array(structure['orientation'])
    path_type = structure['type']
    if path_type == 'SteppedRings':
        path = SteppedRings(size, position, orientation)

        opt = structure['SteppedRings']
        for key in opt.keys():
            if opt[key] is None:
                opt[key] = getattr(path, key)

        path = SteppedRings(size,
                            position,
                            orientation,
                            scale=structure['SteppedRings'],
                            offset=opt['offset'],
                            level_count=opt['level_count'],
                            density=opt['density'],
                            )
    elif path_type == 'InclinedPlane':
        path = InclinedPlane(size, position, orientation)

        opt = structure['InclinedPlane']
        for key in opt.keys():
            if opt[key] is None:
                opt[key] = getattr(path, key)

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
    json_name = sys.argv[1]
    fname = os.path.join(current, json_name)
    with open(fname, "r") as read_file:
        json_structure = json.load(read_file)

    print str(sys.argv[1])
    # get_path_from_json(json_structure)