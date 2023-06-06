function [veh_traj_interact,veh_id_interact] = sametimevehicle(Objectid,frameid,total_veh_traj)
% Check the status information of other vehicles at the same time
veh_traj_same_time = total_veh_traj((total_veh_traj(:,2)==frameid),:);
veh_traj_interact = veh_traj_same_time((veh_traj_same_time(:,1) ~= Objectid),:);
veh_id_interact = unique(veh_traj_interact(:,1));
end