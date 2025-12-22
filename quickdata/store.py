import sqlite3
import json
import time
import threading
from dynamic_reconfigure.server import Server

class ParamStore:
    """
    Persistent parameter storage using SQLite.
    Framework independent (no ROS).
    """

    def __init__( * , self, db_path="quickdata.db", configConfig, callback):
        self.db_path = db_path
        self.configConfig = configConfig
        self.callback = callback
        self._lock = threading.Lock()
        self._init_db()
        self.server = None

    def _init_db(self):
        self.server = Server(self.configConfig, self.callback)
        with sqlite3.connect(self.db_path) as conn:
            conn.execute("""
                CREATE TABLE IF NOT EXISTS parameters (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL,
                    updated_at REAL NOT NULL
                )
            """)
            conn.commit()

    def set(self, key: str, value):
        """Save or update a parameter."""
        payload = json.dumps(value)
        timestamp = time.time()

        with self._lock, sqlite3.connect(self.db_path) as conn:
            conn.execute("""
                INSERT INTO parameters (key, value, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(key)
                DO UPDATE SET
                    value = excluded.value,
                    updated_at = excluded.updated_at
            """, (key, payload, timestamp))
            conn.commit()

    def get(self, key: str, default=None):
        """Retrieve parameter value."""
        with sqlite3.connect(self.db_path) as conn:
            cur = conn.execute(
                "SELECT value FROM parameters WHERE key=?",
                (key,)
            )
            row = cur.fetchone()
            if not row:
                return default
            return json.loads(row[0])

    def update_value(para):
        pass

    def get_all(self):
        """Return all stored parameters."""
        with sqlite3.connect(self.db_path) as conn:
            cur = conn.execute("SELECT key, value FROM parameters")
            return {k: json.loads(v) for k, v in cur.fetchall()}

    def delete(self, key: str):
        """Remove a parameter."""
        with sqlite3.connect(self.db_path) as conn:
            conn.execute("DELETE FROM parameters WHERE key=?", (key,))
            conn.commit()

    def clear(self):
        """Delete all parameters."""
        with sqlite3.connect(self.db_path) as conn:
            conn.execute("DELETE FROM parameters")
            conn.commit()
