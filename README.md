# surface_perception

Simple library for segmentation of tabletop and shelf surfaces.

## Try out the demo
To run the demo, run
`roslaunch surface_perception surface_perceptor.launch target_frame:="gravity_aligned_frame" cloud_in:="your_point_cloud_topic"`
in your terminal.
`target_frame` is the name of a frame whose Z-axis points "up" away from gravity.

Add a Marker topic in RViz to see the segmentation output.
