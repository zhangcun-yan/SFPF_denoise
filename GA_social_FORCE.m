clc;
clear all;
close all;
%——--------------------------——%
global times;
time = 0.04;
global results;
global traj_object_id;
global traj_ebike;
global ebike_id;
global traj_car;
global car_id;
global traj_bike;
global bike_id;
global id;

%% Input the ground truth
ground_truth = csvread('E:\CodeResource\000_Mycode\GA_social_force_model\SFPF_coding\Geroge_ST_trajectory.csv',1,0);
Data = ground_truth;
vehidtotal = unique(Data(:,1));
Data(Data(:,5)<=-6,:)=[];
Data(Data(:,5)>=45,:)=[];
for tttvehid =1:size(vehidtotal,1)
    TTTVEHID = vehidtotal(tttvehid);
    traj_TTTVEHID = Data(Data(:,1)==TTTVEHID,:);
    if size(traj_TTTVEHID,1)<=30
        Data(Data(:,1)==TTTVEHID,:)=[];
    end
end

tf_TRAJ = unique(Data(Data(:,4)>=35,1));
for iii =1:size(tf_TRAJ)
    iiiid =  tf_TRAJ(iii);
    Data(Data(:,1)==iiiid,:)=[];
end
Yolov_raw_ebike = Data(Data(:,3)==22,:);
veh = Data(Data(:,3)==11,:);

% E_bike id
traj_ebike = Data((Data(:,3)==22),:);
traj_ebike(traj_ebike(:,5)>=45,:)=[];
traj_ebike(traj_ebike(:,5)<=-5,:)=[];
ebike_id = unique(traj_ebike(:,1));

% car id
traj_car = Data((Data(:,3)==11),:);
car_id = unique(traj_car(:,1));

% bike id
traj_bike = Data((Data(:,3)==33),:);
traj_bike(traj_bike(:,5)>=45,:)=[];
traj_bike(traj_bike(:,5)<=-5,:)=[];
bike_id = unique(traj_bike(:,1));
%% set the limation of the Parameter Boundary
lb = [0,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100];
ub = [10,100,100,100,100,100,100,100,100,100,100];
initialPopulationMatrix = [5.34065397267225,7.41736759861156,17.3286177581872,-29.1078909609501,-3.69463139412772,-0.254608693364092,4.38373862708397,-4.39016649051955,11.0162337247207,-12.7348665650074,10.1188248462781];

% Vehicle-by-vehicle average calculation
Vhe_num = size(ebike_id,1);

for i = 1:Vhe_num
    id = i;
    object_id = ebike_id(i);
    disp([num2str(i) '/' num2str(Vhe_num)]);
    % Calibration of the same vehicle
    traj_object_id = traj_ebike(traj_ebike(:,1)==object_id,:);
    if  (length(traj_object_id(:,1))<100) continue; %Deleted if the amount of data is less than 10s
    else
        %GA parameter setting
        tic
        options = gaoptimset('PlotFcns',{@gaplotbestf,@gaplotscores},'generation',300, 'InitialPopulation',initialPopulationMatrix,'PopulationSize',100,'TolFun',1e-3,'UseParallel',true);
        results(i,1)=i;
        [results(i,2:12),results(i,13)]=ga(@mainfun_SFC,11,[],[],[],[],lb,ub,[],options);
        toc
    end    
end

%% Result Statistics
result_SCFM=results;
meanSCFM=mean(result_SCFM(:,[2:13]));
stdSCFM=std(result_SCFM(:,[2:13]));
