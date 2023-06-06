function [Force_from_car] = Car_force(current_site,current_speed,car_traj_same_time,car_id_same_time,P)
% Calculate the lateral force from the motor vehicle 
Force_from_car = [];
for i = 1:size(car_id_same_time,1)
    Car_id = car_id_same_time(i);
    Car_site = car_traj_same_time(car_traj_same_time(:,1)==Car_id,4:5);
    Car_speed = car_traj_same_time(car_traj_same_time(:,1)==Car_id,6:7);
    dis_veh_near = sqrt((Car_site(2)-current_site(2))^2+(Car_site(1)-current_site(1))^2);
    Vocter_between_car_bike = current_site - Car_site-current_speed*0.04;
    if dis_veh_near < 8
        Mould_car_veh_speed = norm(Car_speed);
        unit_vocter_between_car_bike = Vocter_between_car_bike./norm(Vocter_between_car_bike);
        % Lateral force action
        Force_from_near_car = -Mould_car_veh_speed*P(10)*exp(-(dis_veh_near-1.4)/P(11))*unit_vocter_between_car_bike;
    else 
        Force_from_near_car = [0,0];
    end
    Force_from_car = [Force_from_car;Force_from_near_car];
end
end