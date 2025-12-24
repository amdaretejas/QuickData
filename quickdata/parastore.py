import sqlite3
import json
import time
import threading
from dynamic_reconfigure.server import Server
from storage import SQLiteParamStore
from copy import deepcopy

class ParamStore:
    def __init__(self, configFile, key, dbName="params.db"):
        self.key = key
        print("This is config file: ")
        self.database = SQLiteParamStore(dbName)
        self.server = Server(configFile, self._callback_function)
        self.default = True

    def _callback_function(self, parameters, level):
        self.config = deepcopy(parameters)
        for values in parameters["groups"]["parameters"]:
            if values != "groups":
                default_value = self.database.get(f"{self.key}.{values}", parameters["groups"]["parameters"][values])
                if parameters["groups"]["parameters"][values] == parameters[values]:
                    parameters[values] = default_value
                if parameters["groups"]["parameters"][values] != default_value:
                    parameters["groups"]["parameters"][values] = default_value

        for values in parameters:
            if values != "groups":
                if parameters["groups"]["parameters"][values] != parameters[values]:
                    self.database.set(f"{self.key}.{values}", parameters[values])
                self.config[values] = self.database.get(f"{self.key}.{values}", parameters["groups"]["parameters"][values])
        return parameters
