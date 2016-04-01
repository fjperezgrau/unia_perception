# unia_perception
ROS package (catkin) for practical training on Computer Vision using OpenCV and PCL, within the Master's Programme on Development and Experimentation of Remotely Piloted Aircraft Systems (International University of Andalusia, Spain).

Author: Francisco Javier Perez Grau (fjperezgrau@gmail.com)

The package is structured as follows:
- launch folder: includes utilities to record and play bag files using an Asus Xtion Pro Live sensor (asus_*.launch), run the image processing node along with image viewers and rviz (img_proc.launch), and run the point cloud processing node along with rviz (pc_proc.launch).
- src folder: includes the node implementations for image processing (img_proc, based on OpenCV) and point cloud processing (pc_proc, based on PCL). The files with 'base' suffix indicate that they are the initial versions before the training starts. The files with 'full' suffix are comprehensive versions with all the topics included in the training.

In order to run, check and adjust the performance of the nodes, a small dataset is provided in https://drive.google.com/file/d/0BytyBmDn9YgsNjlCQVVjc3JlRWs/view?usp=sharing

