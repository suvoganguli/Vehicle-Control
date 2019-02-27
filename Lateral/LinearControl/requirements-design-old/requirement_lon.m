function safe_dist_ft = requirement_lon(v_fps)

mph2fps = 1.47;
fps2mph = 1/mph2fps;

d0 = 20; % ft
slope = 0.6; % (ft/mph)^2
safe_dist_ft = d0 + slope*(v_fps*fps2mph);

end
