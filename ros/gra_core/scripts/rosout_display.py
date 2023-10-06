#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import Int32

class ColorfulLogger:

    NODE_COLORS = [
        "\033[0;91m", "\033[0;92m", "\033[0;93m", "\033[0;94m",  # Non-bold colors for names
        "\033[0;95m", "\033[0;96m", "\033[0;97m", "\033[0;98m"
    ]

    LEVEL_COLOR_MAP = {
        Log.DEBUG: "\033[1;92m",  # Bold + Green
        Log.INFO: "\033[1;94m",  # Bold + Blue
        Log.WARN: "\033[1;93m",  # Bold + Yellow
        Log.ERROR: "\033[1;91m",  # Bold + Red
        Log.FATAL: "\033[1;95m"   # Bold + Magenta
    }

    def __init__(self):
        self.node_color_map = {}
        self.used_colors = set()
        self.lost_sync = 0
        self.begin_time = None

    def _get_color(self, node_name):
        if node_name in self.node_color_map:
            return self.node_color_map[node_name]

        color_idx = hash(node_name) % len(self.NODE_COLORS)
        color = self.NODE_COLORS[color_idx]

        if color in self.used_colors:
            for potential_color in self.NODE_COLORS:
                if potential_color not in self.used_colors:
                    color = potential_color
                    break

        self.used_colors.add(color)
        self.node_color_map[node_name] = color
        return color

    def _get_level_string(self, level):
        return {
            Log.DEBUG: "DEBUG",
            Log.INFO: "INFO",
            Log.WARN: "WARN",
            Log.ERROR: "ERROR",
            Log.FATAL: "FATAL"
        }.get(level, "UNKNOWN")

    def callback(self, data):
        # If string contains "lost sync" (regardless of case), increment counter
        suffix = ""
        lost_sync = ("lost sync" in data.msg.lower())
        if lost_sync:
            self.lost_sync += 1
            suffix = f" ({self.lost_sync} lost syncs)"
        node_color = self._get_color(data.name)
        level_color = self.LEVEL_COLOR_MAP.get(data.level, "\033[1;97m")
        print(f"\033[1;97m[\033[0m{level_color}{self._get_level_string(data.level)}\033[1;97m:\033[0m{node_color}{data.name}\033[1;97m]:\033[0m {data.msg}{suffix}")

    def start(self):
        rospy.init_node('colorful_logger', anonymous=True)
        self.begin_time = rospy.Time.now()
        rospy.Subscriber("/rosout_agg", Log, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = ColorfulLogger()
        logger.start()
    except rospy.ROSInterruptException:
        pass
