# RGBD-Extractor
Easily extract RGB and Scaled Depth images from a ROS bag file (Images dont need any further process)
This scrip gives you images which you can directly feed a 3D registration algorithm with. Depth values are scaled based on regular Xtion pro and Kinect features.

## Two different modes
  * sync = False -> without neet to roscore, extracts images from specified topic with given encoding type
  * sync =True -> as a ros-node uses Message_filter package and synchronises RGB and Depth images and the saves them to spacified directories.

*Note* make sure you set scale and maxDepth variables based on your sensor!!

good Luck 

## To run the code 
run something like: 

for Depth or RGB:
```python
python rgbd-extractor.py numbe_of_frames_to extract --bagfile bagfile_address.bag  --outputA folder_depth_frames --topicA topic_name  --encoding bgr8 or passthrough(depth)
```

for syncing rgb and dpeth and then extracting:
```python
python rgbd-extractor.py 10  --bagfile bagfile_address.bag --outputA folder_depth_frames_A --outputB folder_depth_frames_B --topicA topic_name_A --topicB topic_name_B --sync true
```
