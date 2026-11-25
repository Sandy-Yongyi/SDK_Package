from math import floor


LIDAR_LIST_COUNT = 9
SIDE_DIRECTIONS = ("left", "right")
UPPER_DIRECTIONS = ("left_upper", "right_upper")
SPLIT_BOTTOM_DIRECTIONS = ("left_down", "right_down")
ALL_DIRECTIONS = (*SIDE_DIRECTIONS, *UPPER_DIRECTIONS, "down")
POINT_CLOUD_DIRECTIONS = (*ALL_DIRECTIONS, *SPLIT_BOTTOM_DIRECTIONS)
X_AXIS_DIRECTIONS = (*UPPER_DIRECTIONS, "down", *SPLIT_BOTTOM_DIRECTIONS)

SYSTEM_CONFIG_DIRECTION_KEYS = {
    "left": "left_lidar_ids",
    "right": "right_lidar_ids",
    "left_upper": "left_upper_lidar_ids",
    "right_upper": "right_upper_lidar_ids",
    "down": "down_lidar_ids",
}


def normalize_lidar_ids(lidar_ids):
    if not isinstance(lidar_ids, (list, tuple)):
        return []
    return [str(lidar_id).strip() for lidar_id in lidar_ids if str(lidar_id).strip()]


def filter_active_direction_map(direction_map, directions=ALL_DIRECTIONS):
    active_direction_map = {}
    for direction in directions:
        lidar_ids = normalize_lidar_ids(direction_map.get(direction, []))
        if lidar_ids:
            active_direction_map[direction] = lidar_ids
    return active_direction_map


def get_direction_lidar_config(system_config):
    direction_map = {
        direction: system_config.get(config_key, [])
        for direction, config_key in SYSTEM_CONFIG_DIRECTION_KEYS.items()
    }
    return {
        direction: normalize_lidar_ids(lidar_ids)
        for direction, lidar_ids in direction_map.items()
    }


def get_active_lidar_config(system_config, directions=ALL_DIRECTIONS):
    return filter_active_direction_map(get_direction_lidar_config(system_config), directions)


def get_active_directions(system_config, directions=ALL_DIRECTIONS):
    return list(get_active_lidar_config(system_config, directions))


def get_point_cloud_directions(lidar_directions, translate_data_origin):
    directions = []
    for direction in lidar_directions:
        if direction == "down" and int(translate_data_origin) == 2:
            directions.extend(SPLIT_BOTTOM_DIRECTIONS)
        else:
            directions.append(direction)
    return directions


def get_lidar_filter_bounds(config, direction):
    return tuple(config[f"{direction}_{bound}"] for bound in ("x_min", "x_max", "y_min", "y_max"))


def get_split_bottom_x_grid(config):
    lower = min(config[f"{direction}_x_min"] for direction in SPLIT_BOTTOM_DIRECTIONS)
    upper = max(config[f"{direction}_x_max"] for direction in SPLIT_BOTTOM_DIRECTIONS)
    # 滤波包含最大值，建帧的上界不包含在范围内。
    return floor(lower), floor(upper) + 1, int(config.get("x_threshold", 10))
