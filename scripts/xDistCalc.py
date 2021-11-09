

RL_frame_size = depth2*math.tan(60*pi/180)
RL_pixel_size = RL_frame_size/960

beta = math.atan(bbox_xCen/960)
beta_d = beta*180/pi

x_pixel = depth*math.tan(beta_d)
x = RL_pixel_size*x_pixel
