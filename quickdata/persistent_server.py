import os
import rospy
from dynamic_reconfigure.server import Server
from .storage import SQLiteParamStore


class DRPersistentServer:

    def __init__(self, db_path, config_class, namespace, user_callback=None):

        self.store = SQLiteParamStore(
            os.path.expanduser("~/.ros/" + db_path)
        )

        self.config_class = config_class
        self.namespace = namespace
        self.user_callback = user_callback

        self._ready = False   # 🔥 IMPORTANT

        # Start server
        self.server = Server(self.config_class, self._wrapped_callback)

        # 🔥 ONE-SHOT TIMER — this is the REAL FIX
        rospy.Timer(
            rospy.Duration(0.3),
            self._late_init,
            oneshot=True
        )

    def _late_init(self, event):
        """
        Called AFTER dynamic_reconfigure finishes applying defaults
        """

        values = {}

        for key, default in self.config_class.defaults.items():
            db_key = f"{self.namespace}.{key}"
            values[key] = self.store.get(db_key, default)

        # FINAL override
        self.server.update_configuration(values)

        # Now allow persistence
        self._ready = True

        rospy.loginfo("Dynamic reconfigure initialized with persisted values")

    def _wrapped_callback(self, config, level):

        # ❌ Ignore ALL callbacks until late init finishes
        if not self._ready:
            return config

        # ✅ Persist user changes
        for key, value in config.__dict__.items():
            if not key.startswith("_"):
                self.store.set(f"{self.namespace}.{key}", value)

        if self.user_callback:
            return self.user_callback(config, level)

        return config
