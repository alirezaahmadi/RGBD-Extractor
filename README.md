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

