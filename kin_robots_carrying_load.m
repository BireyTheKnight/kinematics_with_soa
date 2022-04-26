clc; clear all; 
%Adjust the number of parameters according to the number of serial robots using the commented template below the initial parameters
%Link lenghts of the robot arm
%Adjust the number of rows acccording to the number of links of the robot
L{1} = [0 0 3; 
      2 0 0; 
      0 0 3; 
      0 0 2; 
      3 0 0; 
      3 0 0];

%The distance between the last link and the end effector
Lt{1} = [4 0 0];

%The distance between the end effector and the center of the common load 
Lc{1} = [3 2 0];

%The H Matrix representing the rotation axes and directions of translation of joints depending on the joint type of the robot arm with each row corresponding to a joint ([wx wy wz vx vy vz]) 
%Adjust the number of rows acccording to the number of links of the robot
HR{1} = [0 0 1 0 0 0; 
       0 0 1 0 0 0; 
       0 0 1 0 0 0; 
       0 0 0 0 0 1;
       0 0 1 0 0 0; 
       0 0 1 0 0 0];



%Link lenghts of the robot arm
%Adjust the number of rows acccording to the number of links of the robot
L{2} = [0 0 2;  
      0 0 3; 
      0 0 3;
      0 0 3; 
      2 0 0; 
      3 0 0];

%The distance between the last link and the end effector
Lt{2} = [2 0 0];

%The distance between the end effector and the center of the common load 
Lc{2} = [2 1 0];

%The H Matrix representing the rotation axes and directions of translation of joints depending on the joint type of the robot arm with each row corresponding to a joint ([wx wy wz vx vy vz]) 
%Adjust the number of rows acccording to the number of links of the robot
HR{2} = [0 0 1 0 0 0;
       0 0 1 0 0 0; 
       0 0 0 0 0 1;
       0 0 1 0 0 0; 
       0 0 1 0 0 0;
       0 0 1 0 0 0];
   
   
% %Link lenghts of the robot arm
% %Adjust the number of rows acccording to the number of links of the robot
% L{p} = [0 0 3; 
%       2 0 0; 
%       0 0 3; 
%       0 0 2; 
%       3 0 0; 
%       3 0 0];
% 
% %The distance between the last link and the end effector
% Lt{p} = [4 0 0];
% 
% %The distance between the end effector and the center of the common load
% Lc{p} = [3 2 0];
% 
% %The H Matrix representing the rotation axes and directions of translation of joints depending on the joint type of the robot arm with each row corresponding to a joint ([wx wy wz vx vy vz])
% %Adjust the number of rows acccording to the number of links of the robot
% HR{p} = [0 0 1 0 0 0; 
%        0 0 1 0 0 0; 
%        0 0 1 0 0 0; 
%        0 0 0 0 0 1;
%        0 0 1 0 0 0; 
%        0 0 1 0 0 0];

%Mobile platform velocities
V_b = [0; 0; 0; 0; 0; 0];

%Desired common load velocities
V_c = [0; 0; 0; 0; 0; 0];

%Calculating the number of robot arms
p = size(L,2);

%Generation of skew-symmetric l matrices
for i = 1:p
    ln{i} = size(L{i},1);
end

for k = 1:p
    for i = 1:ln{k}
         j = L{k}(i,:);
         j = j';
         Lss{k}{i} = [0, -j(3), j(2); j(3), 0, -j(1); -j(2), j(1), 0];
    end 
end

%Generation of phi matrices 
for j = 1:p
    for i = 1:ln{j}
        R_phis{j}{i+1,i} = [eye(3) zeros(3,3); -Lss{j}{i} eye(3)];
    end
end

%Generation of the remaining phi matrices by the product of the already generated ones
for j = 1:p
    for k = 2:ln{j}
        for i = 1:ln{j}-(k-1)
            R_phis{j}{i+k,i} = R_phis{j}{i+k,i+1}*R_phis{j}{i+1,i};  
        end
    end
end

%Generation of general H matrices for each robot
for i = 1:p
    H{i} = zeros((6*ln{j}),ln{j});
end

for k = 1:p
    j = 1;
    for i = 1:ln{k}
        H{k}(j:j+5,i) = HR{k}(i,:)';
        j = j+6;   
    end
end

%Generation of general phi matrices for each robot
for i = 1:p
    R_phi{i} = eye(6*ln{i});
end
    
for f = 1:p
    a = 3;
    o = 2;
    m = 7;
    for k = 1:6:(5*ln{f})-5
        j = a;
        for i = m:6:(6*ln{f})-5
            R_phi{f}(i:i+5,k:k+5) = R_phis{f}{j,o};
            j = j+1;  
        end
    m = m+6;
    o = o+1;
    a = a+1;
    end
end

%Generation of the matrices that transfer the mobile platform velecities to the links 
for i = 1:p;
    R_phixb{i} = zeros((6*ln{i}),6);    
end

for k = 1:p
    j = 2; 
    for i = 1:6:(6*ln{k})-5
        R_phixb{k}(i:i+5,1:6) = R_phis{k}{j,1};
        j = j+1;
    end
end

%Generation of the matrices that would transfer the final link velocities to the end effectors
for i = 1:p
    Ltss{i} = [0, -Lt{i}(3), Lt{i}(2); Lt{i}(3), 0, -Lt{i}(1); -Lt{i}(2), Lt{i}(1), 0];
end

for i = 1:p
    R_phint{i} = [eye(3) zeros(3,3); -Ltss{i} eye(3)];
end

for i = 1:p
    R_phit{i} = zeros(6,6*ln{i});
end

for i = 1:p
    R_phit{i}(1:6,(6*ln{i})-5:(6*ln{i})) = R_phint{i};
end 

%Generating the matrices that hold the information of the distance between the end effectors and the center of the common load
for i = 1:p
    Lcss{i} = [0, -Lc{i}(3), Lc{i}(2); Lc{i}(3), 0, -Lc{i}(1); -Lc{i}(2), Lc{i}(1), 0];
end

for i = 1:p
    R_phitc{i} = [eye(3) zeros(3,3); -Lcss{i} eye(3)];
end
    
%Generating the matrices that hold the information of the distance between the end effectors and the mobile platform
for i = 1:p
    R_phitb{i} = R_phit{i}*R_phixb{i};
end

%Generating the jacobian matrices for each robot
for i = 1:p
    J_all{i} = R_phit{i}*R_phi{i}*H{i};
end

%Generating the general matrix that holds the information of the distance between the end effectors and the mobile platform
phi_tb = zeros(6*p,6);

j = 1; 
for i = 1:6:(6*p)-5
    phi_tb(i:i+5,1:6) = R_phitb{j};
    j = j+1;
end

%Generating the general matrix that holds the information of the distance between the end effectors and the center of the common load
phi_tc = zeros(6*p,6);

j = 1; 
for i = 1:6:(6*p)-5
    phi_tc(i:i+5,1:6) = R_phitc{j};
    j = j+1;
end

%Generating the general jacobian of a system with p robot arms
len = 0;
for i = 1:p
    len = len + ln{i};
end

J = zeros(6*p,len);

j = 1;
k = 0;
for i = 1:p
    J(j:j+5,k+1:k+size(J_all{i},2)) = J_all{i};
    j = j+6;
    k = k + size(J_all{i},2);
end

%Calculating the desired joint variables

theta_d = pinv(J)*(phi_tc*V_c - phi_tb*V_b)









