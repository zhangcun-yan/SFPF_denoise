function [Rmse] = mainfun_SFC(P)
%% Calibration of social force model parameters
% P = [5.34065397267225,7.41736759861156,17.3286177581872,-29.1078909609501,-3.69463139412772,-0.254608693364092,4.38373862708397,-4.39016649051955,11.0162337247207,-12.7348665650074,10.1188248462781];
Q = [8.5,0.02,0.05,0.05,0.02,0.35];
%% Importing initial parameters
% Boundary parameters
Boundary = [15,-10;25,50;25,-10;35,50];
global frame_step;
frame_step = 0.04;
global traj_object_id;
global traj_ebike;
global ebike_id;
global traj_car;
global car_id;
global traj_bike;
global bike_id;
global object_id;
global id;
global t;
t = 0.04;
%% Vehicle-by-vehicle trajectory reconstruction
sim_all_ebike_traj = [];
% for id = 1:size(ebike_id,1)
object_id = ebike_id(id)

traj_object_id = traj_ebike(traj_ebike(:,1)==object_id,:);
orginal_state = traj_object_id(1,:);
orginal_site = orginal_state(1,4:5);
end_state = traj_object_id(size(traj_object_id,1),:);
Sim_traj_id = [];
%% Get the initial state of the vehicle
vehicle_type = orginal_state(3);
Sim_traj_id = [Sim_traj_id;orginal_state];
i = size(Sim_traj_id,1);
current_site = Sim_traj_id(size(Sim_traj_id,1),4:5);
Destinations = end_state(:,4:5);
% while current_site(2)<Destinations(2) && i<size(traj_object_id,1)
while i < size(traj_object_id,1)
    frame_time = Sim_traj_id(size(Sim_traj_id,1),2);
    % Other vehicles at the same moment
    [Ebike_traj_interact,Ebike_id_interact] = sametimevehicle(object_id,frame_time,traj_ebike);
    [Bike_traj_interact,Bike_id_interact] = sametimevehicle(object_id,frame_time,traj_bike);
    [Car_traj_interact,Car_id_interact] = sametimevehicle(object_id,frame_time,traj_car);
    % Vehicle Status
    current_speed = Sim_traj_id(size(Sim_traj_id,1),6:7);
%     Direction_object = current_speed./norm(current_speed);
        % Desired average speed
        time = size(traj_object_id,1)*0.04; 
        Expect_acc = 1.1*(Destinations-orginal_site)/time;
        % Computing Drivers
        Force_driving = Drving_force(Destinations,current_site,current_speed,P,Expect_acc)*Q(1);%
        % Calculation of boundary forces
        Force_bound = Boundary_force(current_site,current_speed,Expect_acc,Boundary,P,t);
        Force_bound_total = sum(Force_bound,1)*Q(2);%
    % Calculation of interaction forces with electric vehicles
    if size(Ebike_traj_interact,1) > 0
        Force_from_ebike = Ebike_force(current_site,current_speed,Ebike_traj_interact,Ebike_id_interact,frame_step,P);
    else
        Force_from_ebike = [0,0];
    end
    Force_from_ebike_total = sum(Force_from_ebike,1)*Q(3);
    % Calculating the interaction forces with the bike
    if size(Bike_traj_interact,1) > 0
        Force_from_bike = Ebike_force(current_site,current_speed,Bike_traj_interact,Bike_id_interact,frame_step,P);
    else
        Force_from_bike = [0,0];
    end
    Force_from_bike_total = sum(Force_from_bike,1)*Q(4);%
    % Calculation of interaction forces with motor vehicles
    if size(Car_traj_interact,1) > 0
        Force_from_car = Car_force(current_site,current_speed,Car_traj_interact,Car_id_interact,P);
    else
        Force_from_car = [0,0];
    end
    Force_from_car_total = sum(Force_from_car,1)*Q(5);%
    Force = (Force_driving + Force_bound_total + Force_from_ebike_total + Force_from_bike_total + Force_from_car_total)*Q(6);%
    %% Update Status
    Sim_traj_id(i+1,1) = object_id;
    Sim_traj_id(i+1,2) = frame_time+1;
    Sim_traj_id(i+1,3) = vehicle_type;
    Sim_traj_id(i,8:9) =  Force;
    Sim_traj_id(i+1,6:7) = Sim_traj_id(i,6:7)+ Force*frame_step;
    Sim_traj_id(i+1,4:5) = Sim_traj_id(i,4:5)+ Sim_traj_id(i,6:7)*frame_step + 0.5*(frame_step^2)*Force;
    i = i+1;
    current_site = Sim_traj_id(size(Sim_traj_id,1),4:5);
end
Rmse = TRAJRMSE(traj_object_id,Sim_traj_id);
end