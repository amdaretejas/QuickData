import rospy
from dynamic_reconfigure.server import Server
from .storage import SQLiteParamStore
import os


class DRPersistentServer:

    def __init__(self, db_path, config_class, namespace, user_callback=None):

        # Stable DB path (VERY IMPORTANT)
        self.store = SQLiteParamStore(
            os.path.expanduser("~/.ros/" + db_path)
        )

        self.config_class = config_class
        self.namespace = namespace
        self.user_callback = user_callback

        self._initialized = False   # 🔥 KEY FLAG

        # Start server
        self.server = Server(self.config_class, self._wrapped_callback)

        # Let DR finish its internal init
        rospy.sleep(0.2)

        # Load DB and inject values
        self._load_and_apply()

        # Mark system ready
        self._initialized = True

    def _load_and_apply(self):
        values = {}

        for key, default in self.config_class.defaults.items():
            db_key = f"{self.namespace}.{key}"
            values[key] = self.store.get(db_key, default)

        self.server.update_configuration(values)

    def _wrapped_callback(self, config, level):

        # ❌ Ignore callbacks before init is complete
        if not self._initialized:
            return config

        # ✅ Persist values AFTER init
        for key, value in config.__dict__.items():
            if not key.startswith("_"):
                self.store.set(f"{self.namespace}.{key}", value)

        # Call user callback
        if self.user_callback:
            return self.user_callback(config, level)

        return config
