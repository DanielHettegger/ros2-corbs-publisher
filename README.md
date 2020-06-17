# ros2-corbs-publisher
ROS2 package for publishing CoRBS RGB-D benchmark data

See the [CoRBS Website](http://corbs.dfki.uni-kl.de/) for further information on the benchmark. 

## Running the node
Use the following command to run the node 

`ros2 run ros2_corbs_publisher ros2_corbs_publisher_node --ros-args -p data_path:="/path/to/your/data/"`

Or just navigate to the data folder and run the node from there.

## Requirements
* OpenCV
* ROS2