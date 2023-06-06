function [Sim_trajectory] = SFTRAJECTORY(Input_of_one_loop,id)
% Importing raw data requires only one complete trace to be output
% Import Parameters
global P;
% Genetic algorithm calibration results
P = [5.34065397267225,7.41736759861156,17.3286177581872,-29.1078909609501,-3.69463139412772,-0.254608693364092,4.38373862708397,-4.39016649051955,11.0162337247207,-12.7348665650074,10.1188248462781];
% Boundary parameters
Boundary = [13,-35;28,60;25,-10;38,60];
global frame_step;frame_step = 0.04;
[traj_ebike,ebike_id,traj_car,~,traj_bike,~] = PFtrajclass(Input_of_one_loop);
%% Trajectory reconstruction on a vehicle-by-vehicle basis
object_id = ebike_id(id);
traj_object_id = traj_ebike(traj_ebike(:,1)==object_id,:);
% Get the initial state of the vehicle
orginal_state = traj_object_id(1,:);
orginal_site = orginal_state(1,4:5);
vehicle_type = orginal_state(3); 
% Destination
end_state = traj_object_id(size(traj_object_id,1),:);
%% Generate track storage matrix
Sim_traj_id = zeros(size(traj_object_id,1),size(traj_object_id,2));
Sim_traj_id(1,:) = orginal_state;
i = 1;
current_site = Sim_traj_id(i,4:5);
Destinations = end_state(:,4:5);
while  i<size(traj_object_id,1)
    frame_time = Sim_traj_id(i,2);
    %% Other vehicles at the same moment
    [Ebike_traj_sameframe,Ebike_id_sameframe] = sametimevehicle(object_id,frame_time,traj_ebike);
    [Bike_traj_sameframe,Bike_id_sameframe] = sametimevehicle(object_id,frame_time,traj_bike);
    [Car_traj_sameframe,Car_id_sameframe] = sametimevehicle(object_id,frame_time,traj_car);   
    %% Vehicle Status
    current_speed = Sim_traj_id(i,6:7);
    % Desired average speed
    time = size(traj_object_id,1)*0.04;
    Expect_acc = 1.2*(Destinations-orginal_site)/time;
    % Calculate Drivers
    Force_driving = Drving_force(Destinations,current_site,current_speed,P,Expect_acc)*8.5;
    % Calculate the boundary forces
    Force_bound = Boundary_force(current_site,current_speed,Expect_acc,Boundary,P,frame_step);
    Force_bound_total = sum(Force_bound,1)*0.02;
    % Calculate the interaction forces with electric vehicles
    if size(Ebike_traj_sameframe,1) > 0
        Force_from_ebike = Ebike_force(current_site,current_speed,Ebike_traj_sameframe,Ebike_id_sameframe,frame_step,P);
    else
        Force_from_ebike = [0,0];
    end
    Force_from_ebike_total = sum(Force_from_ebike,1);   
    % Calculate the force of interaction with the bike
    if size(Bike_traj_sameframe,1) > 0
        Force_from_bike = Ebike_force(current_site,current_speed,Bike_traj_sameframe,Bike_id_sameframe,frame_step,P);
    else
        Force_from_bike = [0,0];
    end
    Force_from_bike_total = sum(Force_from_bike,1)*0.5;
    % Calculate the interaction forces with motor vehicles
    if size(Car_traj_sameframe,1) > 0
        Force_from_car = Car_force(current_site,current_speed,Car_traj_sameframe,Car_id_sameframe,P);
    else
        Force_from_car = [0,0];
    end
    Force_from_car_total = sum(Force_from_car,1)*0.01;
    Force = (Force_driving + Force_bound_total + Force_from_ebike_total + Force_from_bike_total + Force_from_car_total)*0.35;
    %% Update Status
    Sim_traj_id(i+1,1) = object_id;
    Sim_traj_id(i+1,2) = frame_time+1;
    Sim_traj_id(i+1,3) = vehicle_type;
    Sim_traj_id(i,8:9) =  Force;
    Sim_traj_id(i+1,6:7) = Sim_traj_id(i,6:7)+ Force*frame_step;
    Sim_traj_id(i+1,4:5) = Sim_traj_id(i,4:5)+ Sim_traj_id(i,6:7)*frame_step + 0.5*(frame_step^2)*Force;
    i = i+1;
    current_site = Sim_traj_id(i,4:5);
end
Sim_trajectory = Sim_traj_id;
end