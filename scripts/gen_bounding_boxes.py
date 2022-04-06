import json

def get_bounding_boxes():
    bounding_box = {}
    folder_id = [("df", range(0, 3)), ("pa", range(0, 2)), ("rj", range(0, 6))]
    for (folder, id_range) in folder_id:
        for id in id_range:
            for n in range(0, 90):
                file_path = f"data/train/{folder}-{id}/cvrp-{id}-{folder}-{n}.json"
                box = calculate_bounding_box(file_path)
                bounding_box[folder] = better_box(bounding_box.get(folder, None), box)
    return bounding_box

def better_box(box1, box2):
    if box1 == None:
        return box2
    min_lat  = min(box1["latitude" ][0], box2["latitude" ][0])
    max_lat  = max(box1["latitude" ][1], box2["latitude" ][1])
    min_long = min(box1["longitude"][0], box2["longitude"][0])
    max_long = max(box1["longitude"][1], box2["longitude"][1])
    return {"latitude": [min_lat, max_lat], "longitude": [min_long, max_long]}

def calculate_bounding_box(file_path : str) -> dict:

    with open(file_path) as f:
        points = json.load(f)
        
        min_lat = max_lat = points["origin"]["lat"]
        min_lon = max_lon = points["origin"]["lng"]

        for point in points["deliveries"]:
            point_lat = point["point"]["lat"]
            point_lon = point["point"]["lng"]
            
            if min_lat > point_lat:
                min_lat = point_lat
            elif max_lat < point_lat:
                max_lat = point_lat
                
            if min_lon > point_lon:
                min_lon = point_lon
            elif max_lon < point_lon:
                max_lon = point_lon
            
    return {"latitude" : [min_lat, max_lat], "longitude" : [min_lon, max_lon]}

print(get_bounding_boxes())
