# RGBD-Extractor-
Easily extract RGB and Scaled Depth images from a ROS bag file (Images dont need any further process)
This scrip gives you images which you can directly feed a 3D registration algorithm with. Depth values are scaled based on regular Xtion pro and Kinect features.

good Luck 

## To run the code 
run something like: 

```python
  python rgbd-extractor.py "Number of images to extract." "Input ROS bag." "Output directory." "Image topic." "depth: passthrough, colored image: bgr8"
```

Example:

```python
  python rgbd-extractor.py 10 ~/bag_file.bag ~/bag_file/depth /camera/depth/registered passthrough
```

for rgb or depth:
python rgbd-extractor.py numbe_of_frames_to extract --bagfile bagfile_address.bag  --outputA folder_depth_frames --topicA topic_name  --encoding bgr8 or passthrough(depth)

for syncing rgb and dpeth and then extracting:
python rgbd-extractor.py 10  --bagfile  ~/Downloads/Bagfiles/2020-11-12-10-04-00.bag --outputA ~/rgb --outputB ~/Depth --topicB /phenorob_rs_I_right/aligned_depth_to_color/image_raw --topicA /phenorob_rs_I_right/color/image_raw --sync true