from yolo_ros.msg import ContactsList


def projection(xyxy, obj_id, geo_info_at_time_t) :
    
    message = ContactsList()
    message.id = int(obj_id)
    message.status = int(geo_info_at_time_t['yaw'])
    message.latitude = float(xyxy[0])
    message.longitude = float(xyxy[1])
    message.speed = float(xyxy[2])
    message.size = float(xyxy[3])
    
    return message