clc;
clear all;
close all; 

%% input the noise data
YOLOV7_Data = csvread('E:\CodeResource\000_Mycode\GA_social_force_model\SFPF_coding\clear_yolov7_orgina.csv',1,0);
Data = YOLOV7_Data;
%% define the parameter
global time;
time = 0.04;
%% Classify the trajectory by vehicle type
traj_ebike_orginal = Data(Data(:,3)==22,:);

%% Set the outloop number equal to 200 times
% Create an empty matrix for saving the filter result
Traj_loop_all_pfsf = [];
Traj_loop_all_pfsf(1:size(Data,1),:) = Data;
Traj_loop_all_pfsf(1:size(Data,1),size(Data,2)+1) = 0;
Input_of_one_loop = Data;
Time_total = [];

for loop = 1:200
    tic
    disp([num2str(loop) '/' num2str(50)])
    %% Denoise the non-motor vehicle trajectory one by one
    PF_SFSIM_TRAJ2 = Input_of_one_loop(Input_of_one_loop(:,3)==22,:);
    traj_ebike = Input_of_one_loop(Input_of_one_loop(:,3)==22,:);
    Total_ebike_id = unique(traj_ebike(:,1));
    Measure_from_cv_trajectory = Data(Data(:,3)==22,:);
    for id =1:size(Total_ebike_id,1)      
        Traj_loopid_veh_id = [];
        % the vehicle id
        Ebike_id = Total_ebike_id(id); 
        Eibke_traj_origin = Measure_from_cv_trajectory(Measure_from_cv_trajectory(:,1)==Ebike_id,:);
        % Total length of time frames
        Frame_total= unique(Eibke_traj_origin(:,2));
        % Get the orginal state of the vehicle(speed,acceleration and location)
        orginal_state = Eibke_traj_origin(1,:);        
        % Number of particles and simulation parameters
        N=1000; %Number of particles        
        %% The noise distribution, set the coefficients, and after that use randn to sample from the N~(0,1)
        %% orthogonal distribution, set the variance
        noise_s_x = 0.01;
        noise_v_x = 0.05;
        noise_a_x = 0.00;
        noise_s_y = 0.01;
        noise_v_y = 0.05;
        noise_a_y = 0.00;
        noise_CV_traj_x =0.5;
        noise_CV_traj_y =0.5;         
        %% Using the social force model to update the state, here the social force model is encapsulated into         
        %% a function, which can be called directly to generate the trajectory, but each loop needs to pass 
        %% in the updated overall data that can be
        SF_traj_veh_id = SFTRAJECTORY(Input_of_one_loop,id);
        
        %% Defining the Matrix of Storage
        % Storage space definition - real state (position/velocity), particle filter state (position/velocity)
        % observed values (distance/angle)
        M = size(Frame_total,1);
        
        % Particle filtering estimates
        pf_s_x = zeros(1,M);
        pf_v_x = zeros(1,M);
        pf_a_x = zeros(1,M);
        pf_s_y = zeros(1,M);
        pf_v_y = zeros(1,M);
        pf_a_y = zeros(1,M);
        
        % Prediction of sampled particles
        pre_pf_s_x=zeros(N,M);
        pre_pf_v_x=zeros(N,M);
        pre_pf_a_x = zeros(N,M);
        pre_pf_s_y=zeros(N,M);
        pre_pf_v_y=zeros(N,M);
        pre_pf_a_y = zeros(N,M);
        
        % Resampling particles
        resample_pf_s_x = zeros(N,M);
        resample_pf_v_x = zeros(N,M);
        resample_pf_a_x = zeros(N,M);
        resample_pf_s_y = zeros(N,M);
        resample_pf_v_y = zeros(N,M);
        resample_pf_a_y = zeros(N,M);
        
        CV_traj_x_particle=zeros(N,M);
        CV_traj_y_particle=zeros(N,M);
        
        % Particle weights
        weight_particle=zeros(N,M);
        weight_particle1=zeros(N,M);
        weight_particle2=zeros(N,M);
        
        % Measured values (position and angle calculated using CV_traj)
        CV_traj = Eibke_traj_origin(:,4:5);
        
        %% Particle filtering
        % Assigning initial values
        pf_s_x(1)=orginal_state(1,4);
        pf_v_x(1)=orginal_state(1,6);
        pf_a_x(1)=orginal_state(1,8);
        pf_s_y(1)=orginal_state(1,5);
        pf_v_y(1)=orginal_state(1,7);
        pf_a_y(1)=orginal_state(1,9);
        
        % Initialized particle value selection
        for n=1:N
            pre_pf_s_x(n,1)=pf_s_x(1)+noise_s_x*randn(1);
            pre_pf_v_x(n,1)=pf_v_x(1)+noise_v_x*randn(1);
            pre_pf_a_x(n,1)=pf_a_x(1)+noise_a_x*randn(1);
            
            pre_pf_s_y(n,1)=pf_s_y(1)+noise_s_y*randn(1);
            pre_pf_v_y(n,1)=pf_v_y(1)+noise_v_y*randn(1);
            pre_pf_a_y(n,1)=pf_a_y(1)+noise_a_y*randn(1);
            
            resample_pf_s_x(n,1)=orginal_state(1,4)+noise_s_x*randn(1);
            resample_pf_v_x(n,1)=orginal_state(1,6)+noise_v_x*randn(1);
            resample_pf_a_x(n,1)=orginal_state(1,8)+noise_a_x*randn(1);
            
            resample_pf_s_y(n,1)=orginal_state(1,5)+noise_s_y*randn(1);
            resample_pf_v_y(n,1)=orginal_state(1,7)+noise_v_y*randn(1);
            resample_pf_a_y(n,1)=orginal_state(1,9)+noise_a_y*randn(1);
        end
 
        for Frame_id = 2:M
            % disp([num2str(Frame_id) '/' num2str(M)]);
            for n = 1:N
               % sampling
                pre_pf_s_x(n,Frame_id)=resample_pf_s_x(n,Frame_id-1)+resample_pf_v_x(n,Frame_id-1)*time+0.5*SF_traj_veh_id(Frame_id-1,8)*time*time+noise_s_x*randn(1);
                pre_pf_v_x(n,Frame_id)=resample_pf_v_x(n,Frame_id-1)+SF_traj_veh_id(Frame_id-1,8)*time+noise_v_x*randn(1);
                pre_pf_a_x(n,Frame_id)=SF_traj_veh_id(Frame_id-1,8)+noise_a_x*randn(1);
                pre_pf_s_y(n,Frame_id)=resample_pf_s_y(n,Frame_id-1)+resample_pf_v_y(n,Frame_id-1)*time+0.5*SF_traj_veh_id(Frame_id-1,9)*time*time+noise_s_y*randn(1);
                pre_pf_v_y(n,Frame_id)=resample_pf_v_y(n,Frame_id-1)+SF_traj_veh_id(Frame_id-1,9)*time+noise_v_y*randn(1);
                pre_pf_a_y(n,Frame_id)=SF_traj_veh_id(Frame_id-1,9)+noise_a_y*randn(1);               
               % Calculate weights
                err_CV_traj_x = pre_pf_s_x(n,Frame_id) -CV_traj(Frame_id,1);
                err_CV_traj_y = pre_pf_s_y(n,Frame_id) -CV_traj(Frame_id,2);
                weight_particle1(n,Frame_id) = (1/sqrt(noise_CV_traj_x*2 * pi)) * exp(-(err_CV_traj_x)^2 / (2*noise_CV_traj_x));   %求权重
                weight_particle2(n,Frame_id) = (1/sqrt(noise_CV_traj_y*2 * pi)) * exp(-(err_CV_traj_y)^2 / (2*noise_CV_traj_y));   %求权重
                weight_particle(n,Frame_id) = weight_particle1(n,Frame_id)*weight_particle2(n,Frame_id)+1e-99;
            end
            %% Weight normalization
            weight_particle(:,Frame_id)=weight_particle(:,Frame_id)/sum(weight_particle(:,Frame_id));
            part_weight=cumsum(weight_particle(:,Frame_id));
            ut(1)=rand(1)/N;
            kk = 1;
            hp1 = zeros(N,1);
            for n = 1:N
                ut(n)=ut(1)+(n-1)/N;
                while(part_weight(kk)<ut(n))
                    kk = kk + 1;
                end
                hp(n) = kk;
                weight_particle(n,Frame_id)=1/N;
            end           
            % Resampling
            for n = 1:N
                % The new particles
                % disp([num2str(n) '/' num2str(N)]);
                resample_pf_s_x(n,Frame_id) = pre_pf_s_x(hp(n),Frame_id);
                resample_pf_s_y(n,Frame_id) = pre_pf_s_y(hp(n),Frame_id);
                resample_pf_v_x(n,Frame_id) = pre_pf_v_x(hp(n),Frame_id);
                resample_pf_v_y(n,Frame_id) = pre_pf_v_y(hp(n),Frame_id);
                resample_pf_a_x(n,Frame_id) = pre_pf_a_x(hp(n),Frame_id);
                resample_pf_a_y(n,Frame_id) = pre_pf_a_y(hp(n),Frame_id);
            end           
            % Motion state obtained by particle filtering
            pf_s_x(Frame_id)=mean(resample_pf_s_x(:,Frame_id));
            pf_v_x(Frame_id)=mean(resample_pf_v_x(:,Frame_id));
            pf_a_x(Frame_id)=mean(resample_pf_a_x(:,Frame_id));
            pf_s_y(Frame_id)=mean(resample_pf_s_y(:,Frame_id));
            pf_v_y(Frame_id)=mean(resample_pf_v_y(:,Frame_id));
            pf_a_y(Frame_id)=mean(resample_pf_a_y(:,Frame_id));
        end        
        if size(SF_traj_veh_id,1)<=size(Eibke_traj_origin,1)
            Traj_loopid_veh_id(:,1:3) = SF_traj_veh_id(:,1:3);
        else
            Traj_loopid_veh_id(:,1:3) = Eibke_traj_origin(:,1:3);
        end
        Traj_loopid_veh_id(:,1:3) = Eibke_traj_origin(:,1:3);
        Traj_loopid_veh_id(:,4) = pf_s_x';
        Traj_loopid_veh_id(:,5) = pf_s_y';
        Traj_loopid_veh_id(:,6) = pf_v_x';
        Traj_loopid_veh_id(:,7) = pf_v_y';
        Traj_loopid_veh_id(:,8) = pf_a_x';
        Traj_loopid_veh_id(:,9) = pf_a_y';
        Traj_loopid_veh_id(:,10:13) = zeros(M,4);  
        
%         %% show the result
%          if  Ebike_id ==86     
%           t = 1:M;
%           figure;%
%           clf; 
%           hold on;
%           scatter3(Traj_loopid_veh_id(:,4),Traj_loopid_veh_id(:,5),t','m');
%           hold on;
%           scatter3(SF_traj_veh_id(:,4),SF_traj_veh_id(:,5),t','k');
%           hold on;
%           scatter3(CV_traj(:,1),CV_traj(:,2),t','g');
%           legend('PF+SF trajectory','SF trajectory','CV trajectory');
%           xlabel('x (m)'); ylabel('y (m)'); zlabel('Frame(s)');
%           clf; 
%          end    

        % The filtered trajectory will be used to replace the CV trajectory data in the original
        % database at the end of this cycle.
        traj_ebike(traj_ebike(:,1)==Ebike_id,:)=Traj_loopid_veh_id;
        % Restore the trajectory generated by the fusion model to the i-th cycle trajectory list
        PF_SFSIM_TRAJ2(PF_SFSIM_TRAJ2(:,1)==Ebike_id,:) = Traj_loopid_veh_id;
%        toc
        Time_total = [Time_total;toc]
        disp(['Running time:',num2str(toc)]);
    end
    % The input data of the last social force needs to be replaced, and each loop is iterating with new data
    Input_of_one_loop((Input_of_one_loop(:,3)==22),:) = PF_SFSIM_TRAJ2;
    Output_of_one_loop = Input_of_one_loop;      
    % Add a label to the trajectory of the loop iteration
    Output_of_one_loop(:,14)=loop;
    % Update to write a loop input after a loop ends
    Traj_loop_all_pfsf(1+size(Traj_loop_all_pfsf,1):size(Output_of_one_loop,1)+size(Traj_loop_all_pfsf,1),:)=Output_of_one_loop;
end