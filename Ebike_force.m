function [Force_from_ebike] = Ebike_force(current_site,current_speed,ebike_traj_same_time,ebike_id_same_time,frame_step,P)
% Ebike_force 目标车辆与周围交互车辆之间的作用力
Force_from_ebike = [];
Direction_object = current_speed/norm(current_speed);
for i = 1:size(ebike_id_same_time,1)
    Ebike_id = ebike_id_same_time(i);
    % 获取潜在对象的位置
    Ebike_site = ebike_traj_same_time(ebike_traj_same_time(:,1)==Ebike_id,4:5);
    %     hold on;
    %     ABB = [Ebike_site;current_site];
    %     plot(ABB(:,1),ABB(:,2),'-oR','LineWidth',1,'MarkerSize',5,'MarkerEdgeColor','b','MarkerFaceColor','g');
    Ebike_speed = ebike_traj_same_time(ebike_traj_same_time(:,1)==Ebike_id,6:7);
    dis_veh_near = sqrt((Ebike_site(2)-current_site(2))^2+(Ebike_site(1)-current_site(1))^2);
    if dis_veh_near <4
        % 计算潜在交互对象之间的作用力
        Vocter_object_Ebike_site = Ebike_site-current_site;
        Vocter_Ebike_to_object_site = current_site - Ebike_site;
        %        hold on;
        %        AB = [Ebike_site;current_site];
        %        plot(AB(:,1),AB(:,2),'-ob','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','b','MarkerFaceColor','g');
        Unit_Vocter_betw_object_inter = Vocter_Ebike_to_object_site./norm(Vocter_Ebike_to_object_site);
        Vocter_Relative_speed = current_speed - Ebike_speed;
        Mould_Relative_speed_t = norm(Vocter_Relative_speed*frame_step);
        Vocter_relative_site = Vocter_object_Ebike_site - Vocter_Relative_speed*frame_step;
        Mould_Vocter_relative_site = norm(Vocter_relative_site);
        % 计算短半轴长度
        B_distance = 0.5*sqrt((norm(Vocter_object_Ebike_site)+Mould_Vocter_relative_site)^2-Mould_Relative_speed_t^2);
        % 排斥力
        Repulsive_force = -P(6)*exp(-(B_distance-0.8)/P(7))*Unit_Vocter_betw_object_inter;
        %排斥力的
        Mould_Repulsive_force = norm(Repulsive_force);
        %计算排斥力的垂直向量
        Vocter_Vertical_Repulsive_force_1 = [-(Repulsive_force(2)/Repulsive_force(1))*sqrt(Repulsive_force(1)^2/(Repulsive_force(1)^2+Repulsive_force(2)^2)),sqrt(Repulsive_force(1)^2/(Repulsive_force(1)^2+Repulsive_force(2)^2))];
        Vocter_Vertical_Repulsive_force_2 = [(Repulsive_force(2)/Repulsive_force(1))*sqrt(Repulsive_force(1)^2/(Repulsive_force(1)^2+Repulsive_force(2)^2)),-sqrt(Repulsive_force(1)^2/(Repulsive_force(1)^2+Repulsive_force(2)^2))];
        %计算Vocter_Relative_speed与排斥力垂直向量的夹角
        angle_Vertical_Repulsive__1 = acosd(dot(Vocter_Vertical_Repulsive_force_1,Vocter_Relative_speed)/(norm(Vocter_Vertical_Repulsive_force_1)*norm(Vocter_Relative_speed)));
        angle_Vertical_Repulsive__2 = acosd(dot(Vocter_Vertical_Repulsive_force_2,Vocter_Relative_speed)/(norm(Vocter_Vertical_Repulsive_force_2)*norm(Vocter_Relative_speed)));
        if angle_Vertical_Repulsive__1 > angle_Vertical_Repulsive__2
            Vocter_Vertical_Repulsive_force = Vocter_Vertical_Repulsive_force_2;
        else
            Vocter_Vertical_Repulsive_force = Vocter_Vertical_Repulsive_force_1;
        end
        %        Unit_vocter_Vertical = [exp((Repulsive_force(2))^2/((Repulsive_force(2))^2-(Repulsive_force(1))^2)),exp((Repulsive_force(1))^2/((Repulsive_force(1))^2-(Repulsive_force(2))^2))];
        %
        Avoid_force =P(8)*Mould_Repulsive_force*Vocter_Vertical_Repulsive_force;
        %交互合力
        Direction_angle_cosine = dot(Direction_object, Vocter_object_Ebike_site)./(norm(Direction_object) + norm(Vocter_object_Ebike_site));
        %各向异性系数，判断依据在前后
        if Ebike_site(2)>current_site(2)
            lamuda =1;
        else
            lamuda =0;
        end
        Directional_weights = (P(9) + (1-P(9)) * ((1 + Direction_angle_cosine) / 2))*lamuda;
        Force_bicycle = Directional_weights * Avoid_force + (1-Directional_weights) * Repulsive_force;
    else
        Force_bicycle = [0,0];
    end
    %计算车辆所受到自行车的社会力量的合力
    Force_from_ebike = [Force_from_ebike;Force_bicycle];
end
end