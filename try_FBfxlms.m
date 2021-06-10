% Modify Agustinus Oey's FXLMS simulation.
% Add a Feedback compensation controller D(z) to cancel the feedback
% to noise detecting microphone resulting from canceling speaker.
% It is called FBFXLMS Algorithm.
% Albert Hsu 2016/3/11 (AlbertHsu82@gmail.com)

%--------------------------------------------------------------------------
% One of my friends asked me about the FxLMS algorithm. So, in return, 
% I provided him a little example of a single channel feed-forward active 
% noise control system based on the FxLMS. You can find many good 
% information in "Active Noise Control Systems - Algorithms and DSP 
% Implementations," written by S. M. Kuo and D. R. Morgan in 1996.
%
% Here is the sketch of the system. 
%
%                           +-----------+
%         ----------------- |    F(z)   | <-----
%         |                 +-----------+      |
%         |                                    |
%        +v    +-----------+                   |   +   
% x(k) ---+--->|   P(z)    |--yp(k)----------------> sum --+---> e(k)
%         |    +-----------+                   |      ^-   |
%         | -              +-----------+       |      |    |
%         + <--------------|    Dh(z)  |<------|------|    |
%         |        \       +-----------+            ys(k)  |
%         |                                           |    |
%         |    +-----------+          +-----------+   |    |
%         +--->|   C(z)    |--yw(k)-->|   S(z)    |---+    |
%         |    +-----------+          +-----------+        |
%         |            \                                   |
%         |             \----------------\                 |
%         |                               \                |
%         |    +-----------+          +-----------+        |
%         +--->|   Sh(z)   |--xs(k)-->|    LMS    |<-------+
%              +-----------+          +-----------+        
% 
% I used FIR filter to model P(z), C(z), S(z), and Sh(z).
% 
% Imagine that the noise x(k) is propagating from the source to the sensor,
% through the fluid medium P(z). The sensor measures the arriving noise as 
% yp(k). 
%
% To reduce noise, we generate another 'noise' yw(k) using the controller 
% C(z). We hope that it destructively interferes x(k). It means that the 
% controller has to be a model of the propagation medium P(z). Least mean 
% square algorithm is applied to adjust the controller coefficient/weight.
%
% However, there is also fluid medium S(z) that stay between the actuator 
% and sensor. We called it the secondary propagation path. So, to make the 
% solusion right, we need to compensate the adjustment process using Sh(z), 
% which is an estimate of S(z).
% 
% Let's start the code :)
%
% Developed by Agustinus Oey <oeyaugust@gmail.com>                        
% Center of Noise and Vibration Control (NoViC)                           
% Department of Mechanical Engineering                                    
% Korea Advanced Institute of Science and Technology (KAIST)              
% Daejeon, South Korea 
%--------------------------------------------------------------------------

% Set simulation duration (normalized) 
clear
T=1000; 

% We do not know P(z) and S(z) in reality. So we have to make dummy paths
% Same is the feedback path function F(z) 
Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01];
Sw=Pw*0.25;
Fw=0.058*[0.03 0.21 0.6 1.2 0.6 0.21 0.03]; %F(z) in above figure

% Remember that the first task is to estimate S(z). So, we can generate a
% white noise signal,
x_iden=randn(1,T);

% send it to the actuator, and measure it at the sensor position, 
y_iden=filter(Sw, 1, x_iden);

%Also send it ti the actuator, and measure it at the noise input microphone
y_fb_iden=filter(Fw, 1, x_iden);

% Then, start the identification process
Shx=zeros(1,16);      % the state of Sh(z)
Shw=zeros(1,16);      % the weight of Sh(z)
Dhx=zeros(1,16);      % the state of Dh(z)
Dhw=zeros(1,16);      % the weight of Dh(z)
e_iden=zeros(1,T);    % data buffer for the identification error
e_fb_iden=zeros(1,T); % data buffer for the feedback path identification error

% Identify(Estimate) the feedback function(Dhw) and second path delay(Shw)   
% and apply least mean square algorithm
mu=0.1;                         % learning rate
for k=1:T,                      % discrete time k
    Shx=[x_iden(k) Shx(1:15)];  % update the state
    Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    ShF=sum(Shx.*Dhw);          % calculate output of Dh(z)
    e_iden(k)=y_iden(k)-Shy;    % calculate error    
    e_fb_iden(k)=y_fb_iden(k)-ShF;    % calculate error
    Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
    Dhw=Dhw+mu*e_fb_iden(k)*Shx;   % adjust the weight
end

% Lets check the result
subplot(4,1,1)
plot([1:T], e_iden)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Identification error');
subplot(4,1,2)
stem(Sw) 
hold on 
stem(Shw, 'r*')
ylabel('Amplitude');
xlabel('Numbering of filter tap');
legend('Coefficients of S(z)', 'Coefficients of Sh(z)')

subplot(4,1,3)
plot([1:T], e_fb_iden)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Identification error (FeedBack)');
subplot(4,1,4)
stem(Fw) 
hold on 
stem(Dhw, 'r*')
ylabel('Amplitude');
xlabel('Numbering of filter tap');
legend('Coefficients of F(z)', 'Coefficients of Dh(z)')


% The second task is the active control itself. Again, we need to simulate 
% the actual condition. In practice, it should be an iterative process of
% 'measure', 'control', and 'adjust'; sample by sample. Now, let's generate 
% the noise: 
X=randn(1,T);

% and measure the arriving noise at the sensor position,
Yd=filter(Pw, 1, X);

  
% Initiate the system,
Cx=zeros(1,16);       % the state of C(z)
Fx=zeros(1,16);       % the state of D(z)
Cw=zeros(1,16);       % the weight of C(z)
Sx=zeros(size(Sw));   % the dummy state for the secondary path
e_cont=zeros(1,T);    % data buffer for the control error
Xhx=zeros(1,16);      % the state of the filtered x(k)
Cys=zeros(1,16);      % the state of output y
Fb=0;
Cy=0; 
Shx=zeros(1,16); %Add: Albert's test
X_f=zeros(1,T);

% and apply the FxLMS algorithm
mu=0.1;                            % learning rate
for k=1:T,                         % discrete time k
    X(k) = X(k) + Fb;              % simulate feedback noise plus noise
    Fx=[Cy Fx(1:15)];              % update the fb controller's state
    X_f(k) = X(k) - sum(Dhw.*Fx);  % calculate feedback free reference x
    Cx=[X_f(k) Cx(1:15)];            % update the controller state    
    Cy=sum(Cx.*Cw);                % calculate the controller output	
    Sx=[Cy Sx(1:length(Sx)-1)];    % propagate to secondary path
    e_cont(k)=Yd(k)-sum(Sx.*Sw);   % measure the residue
    Shx=[X_f(k) Shx(1:15)];          % update the state of Sh(z)
    Xhx=[sum(Shx.*Shw) Xhx(1:15)]; % calculate the filtered x(k)
    Cw=Cw+mu*e_cont(k)*Xhx;        % adjust the controller weight
    Fb = sum(Sx.*Fw);              % update Feedback noise
end

% Report the result
figure
subplot(2,1,1)
plot([1:T], e_cont)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise residue')
subplot(2,1,2)
plot([1:T], Yd) 
hold on 
plot([1:T], Yd-e_cont, 'r:')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise signal', 'Control signal')
