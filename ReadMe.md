# Heightmap navigation

## msg
HeightMap
```
std_msgs/Header header
float32 resolution_x
float32 resolution_y
sensor_msgs/Image map
```

## sample launch
```roslaunch heightmap_navigation heightmap_navigation.launch robot_frame:=real/base_link map_frame:=robot_map```