function [traj_ebike,ebike_id,traj_car,car_id,traj_bike,bike_id] = PFtrajclass(Data)
%% Extract vehicle track dataset by vehicle type
% Electric vehicle id
traj_ebike = Data((Data(:,3)==22),:);
ebike_id = unique(traj_ebike(:,1));
% Car id
traj_car = Data((Data(:,3)==11),:);
car_id = unique(traj_car(:,1));
% Bike ID
traj_bike = Data((Data(:,3)==33),:);
bike_id = unique(traj_bike(:,1));
end