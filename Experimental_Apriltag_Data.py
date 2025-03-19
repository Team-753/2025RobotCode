# tag_data.py

# This module contains tag coordinate data and a function to retrieve it.
# To use get_tag_data in another file, simply import it:
#     from tag_data import get_tag_data
# Then call it with a tag ID and a position ("Center", "Left", or "Right"):
#     coordinate = get_tag_data(17, "Center")

TAG_DATA = {
    17: {"Center": (3.84487, 2.91062), "Right": (3.98343, 2.83062), "Left": (3.70631, 2.99062)},
    18: {"Center": (3.20041, 4.0259),  "Right": (3.20041, 3.8659), "Left": (3.20041, 4.1859)},
    19: {"Center": (3.84487, 5.14118), "Right": (3.70631, 5.06118), "Left": (3.98343, 5.22118)},
    20: {"Center": (5.13379, 5.14118), "Right": (4.99523, 5.22118), "Left": (5.27235, 5.06118)},
    21: {"Center": (5.778246, 4.0259), "Right": (5.778246, 4.1859), "Left": (5.778246, 3.8659)},
    22: {"Center": (5.13379, 2.91062), "Right": (5.27235, 2.99062), "Left": (4.99523, 2.83062)},
    6:  {"Center": (13.70336, 2.91062), "Right": (13.84192, 2.99062), "Left": (13.5648, 2.83062)},
    7:  {"Center": (14.347816, 4.0259), "Right": (14.347816, 4.1859), "Left": (14.347816, 3.8659)},
    8:  {"Center": (13.70336, 5.14118), "Right": (13.56480, 5.22118), "Left": (13.84192, 5.06118)},
    9:  {"Center": (12.41444, 5.14118), "Right": (12.27588, 5.06118), "Left": (12.553, 5.22118)},
    10: {"Center": (11.76998, 4.0259), "Right": (11.76998, 3.8659), "Left": (11.76998, 4.1859)},
    11: {"Center": (12.41444, 2.91062), "Right": (12.553, 2.83062), "Left": (12.27588, 2.99062)}
}

def get_tag_data(tag_id, position):
    """
    Retrieve the coordinate tuple for a given tag ID and position.

    Parameters:
        tag_id (int): The tag identifier.
        position (str): The position to query ("Center", "Left", or "Right").

    Returns:
        tuple: The (x, y) coordinate if found, or None if the tag or position is invalid.
    """
    tag_info = TAG_DATA.get(tag_id)
    if tag_info is None:
        return None
    return tag_info.get(position)
