function [Ebike_traj,Ebike_id,bike_traj,bike_id,car_traj,car_id] = Interactveh(object_id,frame_id,traj_ebike,traj_bike,traj_car)
% A summary of this function is shown here
 %% Calculate the status of all vehicles in the system for the same time frame
[Ebike_traj,Ebike_id] = sametimevehicle(object_id,frame_id,traj_ebike);
[bike_traj,bike_id] = sametimevehicle(object_id,frame_id,traj_bike);
[car_traj,car_id] = sametimevehicle(object_id,frame_id,traj_car);
end

