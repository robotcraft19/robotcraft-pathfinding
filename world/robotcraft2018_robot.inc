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

define robotcraft2018_robot position(

  odom_error [0.03 0.03 0.00 0.05]
  localization "odom"
  gui_nose 1
  drive "diff"
  color "green"
  
  size [ 0.122 0.122 0.1 ]
  
  # 1st board:
  block( 
        color "white"
        points 16
        point[0] [ 0.225 0.000 ]
        point[1] [ 0.208 0.086 ]
        point[2] [ 0.159 0.159 ]
        point[3] [ 0.086 0.208 ]
        point[4] [ 0.000 0.225 ]
        point[5] [ -0.086 0.208 ]
        point[6] [ -0.159 0.159 ]
        point[7] [ -0.208 0.086 ]
        point[8] [ -0.225 0.000 ]
        point[9] [ -0.208 -0.086 ]
        point[10] [ -0.159 -0.159 ]
        point[11] [ -0.086 -0.208 ]
        point[12] [ -0.000 -0.225 ]
        point[13] [ 0.086 -0.208 ]
        point[14] [ 0.159 -0.159 ]
        point[15] [ 0.208 -0.086 ]
        z [0.020 0.021]
  )  
  
  
  # 2nd board:
  block( 
        color "white"
        points 16
        point[0] [ 0.225 0.000 ]
        point[1] [ 0.208 0.086 ]
        point[2] [ 0.159 0.159 ]
        point[3] [ 0.086 0.208 ]
        point[4] [ 0.000 0.225 ]
        point[5] [ -0.086 0.208 ]
        point[6] [ -0.159 0.159 ]
        point[7] [ -0.208 0.086 ]
        point[8] [ -0.225 0.000 ]
        point[9] [ -0.208 -0.086 ]
        point[10] [ -0.159 -0.159 ]
        point[11] [ -0.086 -0.208 ]
        point[12] [ -0.000 -0.225 ]
        point[13] [ 0.086 -0.208 ]
        point[14] [ 0.159 -0.159 ]
        point[15] [ 0.208 -0.086 ]
        z [0.042 0.043]
  )    
  
  
  #front left wheel - midblock:
  block( 
     color "black"
     points 4
     point[0] [0.07 0.165]
     point[1] [0.07 0.195]
     point[2] [-0.07 0.195]
     point[3] [-0.07 0.165]
     z [0 0.024]
  )  
  
  #front left wheel - left part:
  block( 
     color "black"
     points 4
     point[0] [0.085 0.165]
     point[1] [0.085 0.195]
     point[2] [0.07 0.195]
     point[3] [0.07 0.165]
     z [0.004 0.02]
  )    
  
  #front left wheel - right part:
  block( 
     color "black"
     points 4
     point[0] [-0.07 0.165]
     point[1] [-0.07 0.195]
     point[2] [-0.085 0.195]
     point[3] [-0.085 0.165]
     z [0.004 0.02]
  )    
  
  #front right wheel - midblock:
  block( 
     color "black"
     points 4
     point[0] [0.07 -0.165]
     point[1] [0.07 -0.195]
     point[2] [-0.07 -0.195]
     point[3] [-0.07 -0.165]
     z [0 0.024]
  )    
  
  #front right wheel - left part:
  block( 
     color "black"
     points 4
     point[0] [0.085  -0.165]
     point[1] [0.085  -0.195]
     point[2] [0.07 -0.195]
     point[3] [0.07 -0.165]
     z [0.004 0.02]
  )  
  
  #front right wheel - right part:
  block( 
     color "black"
     points 4
     point[0] [-0.07 -0.165]
     point[1] [-0.07 -0.195]
     point[2] [-0.085 -0.195]
     point[3] [-0.085 -0.165]
     z [0.004 0.02]
  ) 
  
  #line sensing (front):
  block( 
     color "white"
     points 4
     point[0] [0.19 -0.03]
     point[1] [0.19 0.03]
     point[2] [0.22 0.03]
     point[3] [0.22 -0.03]
     z [0.01 0.02]
  )   
  
  #line sensing (back):
  block( 
     color "white"
     points 4
     point[0] [-0.19 -0.03]
     point[1] [-0.19 0.03]
     point[2] [-0.22 0.03]
     point[3] [-0.22 -0.03]
     z [0.01 0.02]
  )    
  
  #right pole:
  block( 
     color "white"
     points 4
     point[0] [0.006 -0.174]
     point[1] [0.006 -0.186]
     point[2] [-0.006 -0.186]
     point[3] [-0.006 -0.174]
     z [0.021 0.042]
  )   
  
  #left pole:
  block( 
     color "white"
     points 4
     point[0] [0.006 0.174]
     point[1] [0.006 0.186]
     point[2] [-0.006 0.186]
     point[3] [-0.006 0.174]
     z [0.021 0.042]
  )  
  
  #right back pole:
  block( 
     color "white"
     points 4
     point[0] [-0.142 -0.144]
     point[1] [-0.142 -0.156]
     point[2] [-0.148 -0.156]
     point[3] [-0.148 -0.144]
     z [0.021 0.042]
  ) 
  
  #left back pole:
  block( 
     color "white"
     points 4
     point[0] [-0.148 0.144]
     point[1] [-0.148 0.156]
     point[2] [-0.142 0.156]
     point[3] [-0.142 0.144]
     z [0.021 0.042]
  )    
  
  #battery:
  block( 
     color "purple"
     points 4
     point[0] [0.155 0.065]
     point[1] [0.155 -0.065]
     point[2] [-0.211 -0.065]
     point[3] [-0.211 0.065]
     z [0.035 0.041]
  )      
  
  #arduino:
  block( 
     color "LawnGreen"
     points 4
     point[0] [0.06 0.2]
     point[1] [0.06 -0.2]
     point[2] [-0.06 -0.2]
     point[3] [-0.06 0.2]
     z [0.044 0.049]
  )      
      
  IR_sensor(pose [ 0.055 0.0 -0.056 0.0 ])         # front_sensor publishes data at the /base_scan_0 topic
  IR_sensor(pose [ 0.04 0.0425 -0.056 45.0 ])      # left_sensor  publishes data at the /base_scan_1 topic
  IR_sensor(pose [ 0.04 -0.0425 -0.056 -45.0 ])    # right_sensor publishes data at the /base_scan_2 topic
)
