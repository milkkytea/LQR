function SLQR(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.

%   Copyright 2003-2018 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C MEX counterpart: mdlInitializeSizes
%%
function setup(block)
 % 1. Регистрируем 1 рабочую область (память)
% Register number of ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 3;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = 1;
block.InputPort(1).DatatypeID  = 1;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions       = 1;
block.OutputPort(1).DatatypeID  = 1; % double
block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------
  % Обязательно регистрируем метод пост-пропагации:
block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('SetInputPortSamplingMode', @SetPorts);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C MEX counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
    block.NumDworks = 4;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;


  block.Dwork(2).Name            = 'R';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'state_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;

  block.Dwork(4).Name            = 'state_2';
  block.Dwork(4).Dimensions      = 3;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;

%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)

%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C MEX counterpart: mdlStart
%%
function Start(block)

block.Dwork(1).Data = 0;

block.Dwork(2).Data = 1.0;
block.Dwork(3).Data = 0.0;
block.Dwork(4).Data = zeros(3,1);

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
    function Outputs(block)

    Ts = 0.005;

    e     = double(block.InputPort(1).Data);
    e_dot = double(block.InputPort(2).Data);

    A = [1, Ts, 0;
         0, 1,  Ts;
         0, 0,  0];

    B = [0;
         0;
         1.30767676];

    Q = diag([0.1792, 0.0055, 0.0028]);

    %% Параметры адаптации R
    R0    = 2;
    Rspan = 1.7;

    Rmin = R0 - Rspan;
    Rmax = R0 + Rspan;

    e_on  = 0.000010;
    e_off = 0.000004;

    progress_deadband = 0.0002;
    progress_scale    = 0.0001;

    beta_active = 0.6;
    beta_return = 0.0015;

    R_prev = double(block.Dwork(2).Data);

    if ~isfinite(R_prev) || R_prev < Rmin || R_prev > Rmax
        R_prev = R0;
    end


    adaptation_active = logical(block.Dwork(3).Data);

    %% Гистерезис
    if ~adaptation_active && abs(e) > e_on
        adaptation_active = true;
    elseif adaptation_active && abs(e) < e_off
        adaptation_active = false;
    end

    %% Расчёт R
    if adaptation_active

        progress = -e * e_dot;

        if abs(progress) < progress_deadband
            progress = 0;
        end

        R_target = R0 + ...
            Rspan * tanh(progress / progress_scale);
if abs(e_dot) > 0.05 
    R_target = R_target + 0.5; 
end
    if e_dot < 0
        R_target = R_target + 0.004; 
    else
        R_target = R_target - 0.004; 
    end
        R_target = min(max(R_target, Rmin), Rmax);

        R = R_prev + beta_active * (R_target - R_prev);

    else

        R = R_prev + beta_return * (R0 - R_prev);

    end
   
    [K, ~, ~] = dlqr(A, B, Q, R);
    
    %% Сглаживание коэффициентов
    K_prev = double(block.Dwork(4).Data(:)).';
    
    if any(~isfinite(K_prev)) || all(K_prev == 0)
        K_prev = K;
    end
    
    beta_K = 1;
    K_out = K_prev + beta_K * (K - K_prev);
    
    %% Сохранение состояний
    block.Dwork(2).Data = R;
    block.Dwork(3).Data = double(adaptation_active);
    block.Dwork(4).Data = K_out(:);  % теперь Dwork имеет ширину 3
    
    %% Выходы
    for i = 1:3
        block.OutputPort(i).Data = cast(K_out(i), 'single');
    end


%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlUpdate
%%
function Update(block)
block.Dwork(1).Data = cast(block.InputPort(1).Data, "double");

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlDerivatives
%%
function Derivatives(block)

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C MEX counterpart: mdlTerminate
%%
function SetPorts(block, idx, fd)
    block.InputPort(idx).SamplingMode = fd;
    for i = 1:block.NumOutputPorts
        block.OutputPort(i).SamplingMode = fd; 
    end
function Terminate(block)

%end Terminate
