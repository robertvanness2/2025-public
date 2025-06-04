import numpy as np

from utils.field import Field

# Start positions
FIELD_CENTER_START_POSE = (7.2, Field.WIDTH * 0.5, np.pi)
DS_LEFT_START_POSE = (7.2, 5.4, np.pi)
DS_LEFT_WALL_START_POSE = (7.0, 7.5, np.pi)

# Reef faces
FRONT_LEFT_APPOACH_POSE = (3.6, 5.2, np.deg2rad(300))
FRONT_MIDDLE_APPOACH_POSE = (2.9, 4.0, 2 * np.pi)
BACK_LEFT_APPROACH_POSE = (5.3, 5.4, np.deg2rad(240))
BACK_RIGHT_APPROACH_POSE = (5.3, 2.7, np.deg2rad(120))

# Source poses
BACK_CENTER_APPROACH_POSE = (6.0, Field.WIDTH * 0.5, np.pi)
BACK_LEFT_TO_SOURCE_INTERMEDIATE = (4.0, 6.3, np.deg2rad(330))
FRONT_MIDDLE_TO_SOURCE_INTERMEDIATE = (2.4, 5.7, np.deg2rad(330))
TOP_RIGHT_SOURCE = (1.5, 7.4, np.deg2rad(305))

# Return poses
FRONT_LEFT_RETURN_POSE = (2.1, 6.8, np.deg2rad(300))
BACK_LEFT_TO_SOURCE_RETURN_INTERMEDIATE = (2.5, 6.9, np.deg2rad(240))
START_BACK_LEFT_APPROACH_POSE = (6.5, 5.5, np.deg2rad(240))
FRONT_MIDDLE_TO_SOURCE_RETURN = (1.5, 6.5, 1.75 * np.pi)
WALL_START_TO_FRONT = (3.0, 6.0, 1.75 * np.pi)
