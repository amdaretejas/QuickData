import os
import rospy
from dynamic_reconfigure.server import Server
from .storage import SQLiteParamStore


class DRPersistentServer:

    def __init__(self, config_class, namespace, user_callback=None):

        db_path = os.path.expanduser("~/.ros/motion_params.db")
        self.store = SQLiteParamStore(db_path)

        self.config_class = config_class
        self.namespace = namespace
        self.user_callback = user_callback

        self.server = Server(self.config_class, self._wrapped_callback)

        rospy.sleep(0.1)  # 🔥 IMPORTANT
        self._load_and_apply()

    def _load_and_apply(self):

        defaults = {}
        for k, v in self.config_class.defaults.items():
            defaults[k] = v

        values = {}
        for key in defaults:
            db_key = f"{self.namespace}.{key}"
            values[key] = self.store.get(db_key, defaults[key])

        self.server.update_configuration(values)

    def _wrapped_callback(self, config, level):

        for key, value in config.__dict__.items():
            if not key.startswith("_"):
                self.store.set(f"{self.namespace}.{key}", value)

        if self.user_callback:
            return self.user_callback(config, level)

        return config
