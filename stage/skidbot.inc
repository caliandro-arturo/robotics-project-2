define lidar ranger
(
  sensor
  (
    fov 359.0
    samples 640
    range [0.2 10]
  )
  # generic model properties
  color "red"
  size [ 0.06 0.15 0.03 ]
)

define skidbot position
(
  localization "odom"
  odom_error [ 0.0 0.0 0.0 0.0 ]

  size [ 0.60 0.40 0.5 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  lidar(pose [ 0.0 0.0 0.0 0.0 ])
)
