function [Force_drving] = Drving_force(destinations,current_site,current_speed,P,Expect_acc)
%% Calculated Drivers
driving_force = destinations - current_site;
Driving_force_Vectors = driving_force/norm(driving_force);
% Expected acceleration, obtained by fitting the distribution of the true values 
% Expect_acc_x = normrnd(0.2*2, 1 * 0.4);
% Expect_acc_y = normrnd(1.8 * 5.2289, 1 * 1.3464);
% Expect_acc = [Expect_acc_x Expect_acc_y];
% Expect_acc = 12;
Force_drving = (Expect_acc.*Driving_force_Vectors-current_speed)/P(1);
end
