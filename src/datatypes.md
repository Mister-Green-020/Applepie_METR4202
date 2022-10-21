# Connecting data types
For my own personal reference and understanding how to connect data types together efficiently

## FiducialArray
- std_msgs/Header header
    - uint32 seq
    - time stamp
    - string frame_id
- int32 image_seq
- fiducial_msgs/Fiducial[] fiducials
    - int32 fiducial_id
    - int32 direction
    - float64 x0
    - float64 y0
    - float64 x1
    - float64 y1
    - float64 x2
    - float64 y2
    - float64 x3
    - float64 y3

## Pose
- geometry_msgs/Point position
    - float64 x
    - float64 y
    - float64 z
- geometry_msgs/Quaternion orientation
    - float64 x
    - float64 y
    - float64 z
    - float64 w

## Joint States
- std_msgs/Header header
    - uint32 seq
    - time stamp
    - string frame_id
- string[] name
- float64[] position
- float64[] velocity
- float64[] effort

## Image
- std_msgs/Header header
    - uint32 seq
    - time stamp
    - string frame_id
- uint32 height
- uint32 width
- string encoding
- uint8 is_bigendian
- uint32 step
- uint8[] data

## ColorRGBA
- float32 r
- float32 g
- float32 b
- float32 a

## Bool
- bool data
