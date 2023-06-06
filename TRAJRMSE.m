function [rmse] = TRAJRMSE(cvtraj,groundtruth)
%Calculate the RMSE between the processed trajectory and the real trajectory
    if size(groundtruth,1)>size(cvtraj,1)
        groundtruth = groundtruth(1:size(cvtraj,1),:);
    else
        cvtraj = cvtraj(1:size(groundtruth,1),:);
    end
rmse = sqrt(sum((cvtraj(:,4)-groundtruth(:,4)).^2+(cvtraj(:,5)-groundtruth(:,5)).^2)/(size(groundtruth,1)));
end

