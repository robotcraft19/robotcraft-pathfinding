define IR_sensor ranger(

  sensor(
    range [ 0.1  0.8 ]
    fov 4.0
    samples 1
  )

  color "black"
  size [ 0.019 0.045 0.014 ]

  block( points 4
     point[0] [0 0]
     point[1] [0 1]
     point[2] [1 1]
     point[3] [1 0]
     z [0 1]
  ) 
)



define sonar ranger(

  sensor(
    range [ 0.16  2.49 ]
    fov 60.0
    samples 1
  )

  size [ 0.018 0.02 0.024 ]

  block( 
     color "black"
     points 4
     point[0] [0.002 0.018]
     point[1] [0.018 0.018]
     point[2] [0.018 0.002]
     point[3] [0.002 0.002]
     z [0.004 0.02]
  ) 
  
  block( 
     color "SeaGreen"
     points 4
     point[0] [0 0]
     point[1] [0 0.02]
     point[2] [0.002 0.02]
     point[3] [0.002 0]
     z [0 0.024]
  )   
  
)



define robotcraft2017_robot position(

  odom_error [0.03 0.03 0.00 0.05]
  localization "odom"
  gui_nose 1
  drive "diff"
  color "green"
  
  size [ 0.24 0.195 0.13 ]
  
  #robot's main body:
  block( 
     color "LawnGreen"
     points 4
     point[0] [0.225 0.03]
     point[1] [0.225 0.165]
     point[2] [0.015 0.165]
     point[3] [0.015 0.03]
     z [0.02 0.065]
  )   
  
  #robot's top surface:
  block( 
     color "LawnGreen"
     points 4
     point[0] [0.215 0.04]
     point[1] [0.215 0.155]
     point[2] [0.025 0.155]
     point[3] [0.025 0.04]
     z [0.129 0.13]
  )   
  
  #front left wheel - midblock:
  block( 
     color "black"
     points 4
     point[0] [0.225 0.165]
     point[1] [0.225 0.195]
     point[2] [0.18 0.195]
     point[3] [0.18 0.165]
     z [0 0.085]
  )  
  
  #front left wheel - left part:
  block( 
     color "black"
     points 4
     point[0] [0.245 0.165]
     point[1] [0.245 0.195]
     point[2] [0.225 0.195]
     point[3] [0.225 0.165]
     z [0.02 0.065]
  )    
  
  #front left wheel - right part:
  block( 
     color "black"
     points 4
     point[0] [0.18 0.165]
     point[1] [0.18 0.195]
     point[2] [0.16 0.195]
     point[3] [0.16 0.165]
     z [0.02 0.065]
  )   
  
  #front right wheel - midblock:
  block( 
     color "black"
     points 4
     point[0] [0.225 0]
     point[1] [0.225 0.03]
     point[2] [0.18 0.03]
     point[3] [0.18 0]
     z [0 0.085]
  )    
  
  #front right wheel - left part:
  block( 
     color "black"
     points 4
     point[0] [0.245 0]
     point[1] [0.245 0.03]
     point[2] [0.225 0.03]
     point[3] [0.225 0]
     z [0.02 0.065]
  )  
  
  #front right wheel - right part:
  block( 
     color "black"
     points 4
     point[0] [0.18 0]
     point[1] [0.18 0.03]
     point[2] [0.16 0.03]
     point[3] [0.16 0]
     z [0.02 0.065]
  )      
  
  #back left wheel - midpart:
  block( 
     color "black"
     points 4
     point[0] [0.065 0.165]
     point[1] [0.065 0.195]
     point[2] [0.02 0.195]
     point[3] [0.02 0.165]
     z [0 0.085]
  )   
  
  #back left wheel - left part:
  block( 
     color "black"
     points 4
     point[0] [0.085 0.165]
     point[1] [0.085 0.195]
     point[2] [0.065 0.195]
     point[3] [0.065 0.165]
     z [0.02 0.065]
  )    
  
  #back left wheel - right part:
  block( 
     color "black"
     points 4
     point[0] [0.02 0.165]
     point[1] [0.02 0.195]
     point[2] [0 0.195]
     point[3] [0 0.165]
     z [0.02 0.065]
  )      
  
  #back right wheel - midpart:
  block( 
     color "black"
     points 4
     point[0] [0.065 0]
     point[1] [0.065 0.03]
     point[2] [0.02 0.03]
     point[3] [0.02 0]
     z [0 0.085]
  )   
  
  #back right wheel - left part:
  block( 
     color "black"
     points 4
     point[0] [0.085 0]
     point[1] [0.085 0.03]
     point[2] [0.065 0.03]
     point[3] [0.065 0]
     z [0.02 0.065]
  )    
  
  #back right wheel - right part:
  block( 
     color "black"
     points 4
     point[0] [0.02 0]
     point[1] [0.02 0.03]
     point[2] [0 0.03]
     point[3] [0 0]
     z [0.02 0.065]
  )     
  
  #right back pole:
  block( 
     color "gray"
     points 4
     point[0] [0.045 0.05]
     point[1] [0.045 0.055]
     point[2] [0.04 0.055]
     point[3] [0.04 0.05]
     z [0.065 0.129]
  )  
  
  #left back pole:
  block( 
     color "gray"
     points 4
     point[0] [0.045 0.135]
     point[1] [0.045 0.14]
     point[2] [0.04 0.14]
     point[3] [0.04 0.135]
     z [0.065 0.129]
  )   
  
  #right front pole:
  block( 
     color "gray"
     points 4
     point[0] [0.195 0.05]
     point[1] [0.195 0.055]
     point[2] [0.2 0.055]
     point[3] [0.2 0.05]
     z [0.065 0.129]
  )
  
  #left back pole:
  block( 
     color "gray"
     points 4
     point[0] [0.195 0.135]
     point[1] [0.195 0.14]
     point[2] [0.2 0.14]
     point[3] [0.2 0.135]
     z [0.065 0.129]
  )  
  
  IR_sensor(pose [ 0.09 0.0 -0.0145 0.0 ])      # IR_sensor publishes data at the /base_scan_0 topic
  sonar(pose [ 0.09 0.0575 -0.0245 45.0 ])      # left sonar publishes data at the /base_scan_1 topic
  sonar(pose [ 0.09 -0.0575 -0.0245 -45.0 ])    # right sonar publishes data at the /base_scan_2 topic
)
