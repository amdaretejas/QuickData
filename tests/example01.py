from quickdata import ParamStore
import rospy

store = ParamStore("params.db")

def reconfig_cb(config, level):
    store.set("yolo.conf_threshold", config.conf_threshold)
    store.set("yolo.iou_threshold", config.iou_threshold)
    return config

conf = store.get("yolo.conf_threshold", 0.5)
iou  = store.get("yolo.iou_threshold", 0.45)

rospy.set_param("conf_threshold", conf)
rospy.set_param("iou_threshold", iou)

