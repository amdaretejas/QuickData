import sqlite3
import json
import time
import threading


class SQLiteParamStore:
    def __init__(self, db_path):
        self.db_path = db_path
        self._lock = threading.Lock()
        self._init_db()

    def _init_db(self):
        with sqlite3.connect(self.db_path) as conn:
            conn.execute("""
                CREATE TABLE IF NOT EXISTS params (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL,
                    updated REAL NOT NULL
                )
            """)
            conn.commit()

    def set(self, key, value):
        with self._lock, sqlite3.connect(self.db_path) as conn:
            conn.execute("""
                INSERT INTO params (key, value, updated)
                VALUES (?, ?, ?)
                ON CONFLICT(key)
                DO UPDATE SET value=excluded.value, updated=excluded.updated
            """, (key, json.dumps(value), time.time()))
            conn.commit()

    def get(self, key, default=None):
        with sqlite3.connect(self.db_path) as conn:
            cur = conn.execute("SELECT value FROM params WHERE key=?", (key,))
            row = cur.fetchone()
            return default if row is None else json.loads(row[0])

    def get_namespace(self, namespace, defaults):

        values = {}

        for key in defaults:
            full_key = namespace + "." + key
            value = self.get(full_key, defaults[key])
            values[key] = value

        return values
