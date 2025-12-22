from quickdata import ParamStore

store = ParamStore("params.db")

store.set("yolo.conf_threshold", 0.62)
store.set("pid.kp", 1.8)

print(store.get("yolo.conf_threshold"))
