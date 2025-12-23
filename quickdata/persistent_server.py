from dynamic_reconfigure.server import Server
from .storage import SQLiteParamStore


class DRPersistentServer:
    """
    ROS1 Dynamic Reconfigure server with automatic DB persistence.
    """

    def __init__(self, db_path, config_class, namespace, user_callback):
        self.store = SQLiteParamStore(db_path)
        self.config_class = config_class
        self.namespace = namespace
        self.user_callback = user_callback

        # 1️⃣ Start server with wrapped callback
        self.server = Server(self.config_class, self._wrapped_callback)

        # 2️⃣ Load DB values and inject them
        self._load_and_apply()

    def _load_and_apply(self):

        # Step 1: collect defaults
        defaults = {}
        for key in self.config_class.defaults:
            defaults[key] = self.config_class.defaults[key]

        # Step 2: override with DB values
        values = {}
        for key in defaults:
            db_key = self.namespace + "." + key
            values[key] = self.store.get(db_key, defaults[key])

        # Step 3: push into dynamic reconfigure
        self.server.update_configuration(values)


    def _wrapped_callback(self, config, level):
        # Save every parameter automatically
        for key, value in config.__dict__.items():
            if not key.startswith("_"):
                self.store.set(f"{self.namespace}.{key}", value)

        # Call user callback if provided
        if self.user_callback:
            return self.user_callback(config, level)

        return config
