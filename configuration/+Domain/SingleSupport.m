% Single Support 
% 
%       Contacts: Stance Toe, Stance Heel
%       Guard to next domain: swing heel strikes

function domain = SingleSupport(domain, stanceLeg, phaseType)

% Get swing and stance foot positions
if strcmp(stanceLeg,'Right')
    sl='Right'; nsl = 'Left'; 
    p_nsf = domain.getCartesianPosition(domain.ContactPoints.LeftSole); 
    p_sf  = domain.getCartesianPosition(domain.ContactPoints.RightSole);
else
    sl='Left'; nsl = 'Right';         
    p_nsf = domain.getCartesianPosition(domain.ContactPoints.RightSole); 
    p_sf  = domain.getCartesianPosition(domain.ContactPoints.LeftSole);
end

% Extract state variables
x = domain.States.x;
dx = domain.States.dx;
    
%% Phase Variable and Outputs
switch phaseType
    
    case 'TimeBased'
        
    %%% Relative degree 2 outputs -- joint positions (time-based)
    ya_2    = [x([sl, 'Hip']); x([sl, 'Knee']); x([sl,'Ankle']); x([nsl, 'Hip']), x([nsl, 'Knee']); x([nsl,'Ankle'])];
    y2_label = { [sl, 'Hip'],    [sl, 'Knee'],    [sl,'Ankle'],    [nsl, 'Hip'],    [nsl, 'Knee'],    [nsl,'Ankle']};
        
    % use time as phase variable if time-based
    t = SymVariable('t');
    p = SymVariable('p',[2,1]);
    tau = (t-p(2))/(p(1)-p(2));
    
    case 'StateBased'

    %%% Relative degree 2 outputs -- joint positions (time-based)
    
%     ya_2    = [x([sl, 'Hip']); x([sl, 'Knee']); x([sl,'Ankle']); x([nsl, 'Hip']), x([nsl, 'Knee']); x([nsl,'Ankle'])];
%     y2_label = { [sl, 'Hip'],    [sl, 'Knee'],    [sl,'Ankle'],    [nsl, 'Hip'],    [nsl, 'Knee'],    [nsl,'Ankle']};
    ya_2    = [x([sl, 'Hip']); x([sl, 'Knee']); x([nsl, 'Hip']), x([nsl, 'Knee']); x([nsl,'Ankle'])];
    y2_label = { [sl, 'Hip'],    [sl, 'Knee'],    [nsl, 'Hip'],    [nsl, 'Knee'],    [nsl,'Ankle']};

    % use linearized hip position as tau if phase-based
    hip_pos = domain.getCartesianPosition(domain.Joints(getJointIndices(domain,'RightHip')));
    phip = hip_pos(1) - p_sf(1);
    deltaphip = linearize(phip, x);
    
    %%% define tau(q)
    p = SymVariable('p',[2,1]);
    tau = (deltaphip - p(2))/(p(1)-p(2));
    
    %% Relative degree 1 output: virtual constraints
    v_hip = jacobian(deltaphip, x)*dx;
    y1 = VirtualConstraint(domain,v_hip,'velocity',...
        'DesiredType','Constant',...
        'RelativeDegree',1,...
        'OutputLabel',{'vhip'},...
        'PhaseType', phaseType,...
        'PhaseVariable', tau,...
        'PhaseParams', p,...
        'Holonomic', false);
    domain = addVirtualConstraint(domain,y1);

    otherwise 
        error("phaseType must be 'TimeBased' or 'StateBased'");
end
    
%% Relative degree 2 output: virtual constraints
y2 = VirtualConstraint(domain,ya_2,'position',...
                    'DesiredType','Bezier',...
                    'PolyDegree',5,...
                    'RelativeDegree',2,...
                    'OutputLabel',{y2_label},...
                    'PhaseType',phaseType,...
                    'PhaseVariable',tau,...
                    'PhaseParams',p,...
                    'Holonomic',true);
domain = addVirtualConstraint(domain,y2);
    
%%  Add holonomic constraints
% get foot geometry for toe contact
% geometry = struct('la',0,'lb',0,'La', domain.foot_geometry.length/2, 'Lb',domain.foot_geometry.length/2);
geometry = struct('la',domain.foot_geometry.length/2, 'lb',domain.foot_geometry.length/2);

if strcmp(stanceLeg,'Right')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% Contacts  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add Stance Heel Contact
    frame = ToContactFrame(domain.ContactPoints.RightSole, ...
                            'PlanarLineContactWithFriction');
    domain = addContact(domain, frame, domain.fric_coef, geometry);
    
elseif strcmp(stanceLeg,'Left')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% Contacts  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add Stance Heel Contact
    frame = ToContactFrame(model.ContactPoints.LeftSole, ...
                            'PlanarLineContactWithFriction');
    domain = addContact(domain, frame, domain.fric_coef, geometry);
   
else
    error("stanceLeg must be Left or Right");
end

%%%%%%%%%%%%%%%%%%%%%%%%% Guard Event  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% height of swing heel
h_nsf = UnilateralConstraint(domain,p_nsf(3),'nsf','x');
domain = addEvent(domain, h_nsf);
    
% IMPORTANT FOR TIME-BASED:
if strcmp(stanceLeg,'Right')
    domain.PreProcess = @Sim.NewStepPreProcess;
else
    domain.PreProcess = @Sim.MidStepPreProcess;
end
end
