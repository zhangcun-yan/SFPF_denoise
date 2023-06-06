function [Force_bound] = Boundary_force(current_site,current_speed,Expect_acc,Boundary,P,t)
% Calculate the boundary forces
Force_bound =[];
for i=[1,3]
    slope = (Boundary(i+1,2)-Boundary(i,2))/(Boundary(i+1,1)-Boundary(i,1));
    intercept = (Boundary(i+1,2)*Boundary(i,1)-Boundary(i,2)*Boundary(i+1,1))/(Boundary(i,1)-Boundary(i+1,1));
    angle_bound = atand(slope); %Calculate the angle
    % Calculate the boundary points of the horizontal target
    X_bound = (current_site(2)-intercept)/slope;
    Dis_boundary = current_site(1) - X_bound ;
    Site_bound = [X_bound,current_site(2)];
    % Calculate the boundary points of vertical targets
    Length_X_bound = Dis_boundary;
    X_Vertical_bound = X_bound + Length_X_bound*cosd(angle_bound)*cosd(angle_bound);
    Y_Vertical_bound = current_site(2) +  Length_X_bound*cosd(angle_bound)*sind(angle_bound);
    site_Vertical_bound_object = [X_Vertical_bound,Y_Vertical_bound];
    % Draw a line perpendicular to the boundary past the position point of the target object
    Bound_force_Vectors_XY = current_site-site_Vertical_bound_object-current_speed*t;
    Bound_force_Vectors_uint = Bound_force_Vectors_XY/norm(Bound_force_Vectors_XY);
    %Distance from the boundary action point to the moving target
    Diss_boundary = norm(Bound_force_Vectors_XY); %Boundary forces: outer and inner virtual boundaries   
    r1 = (0.8+norm(current_speed)/norm(Expect_acc))*(0.4+0.8)/2;
    % Make the non-motorized size dynamic, the smaller the speed, the smaller the space
    if i ==1
        Force_boundary = P(2)*exp(-(abs(Diss_boundary)-0.4-r1)/P(3))*Bound_force_Vectors_uint*0.5;
    else
        Force_boundary = P(4)*exp(-(abs(Diss_boundary)-0.4-r1)/P(5))*Bound_force_Vectors_uint*1.5;
    end
    Force_bound = [Force_bound;Force_boundary];
end
end