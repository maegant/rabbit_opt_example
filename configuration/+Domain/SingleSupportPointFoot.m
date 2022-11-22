% Single Support 
% 
%       Contacts: Stance Toe, Stance Heel
%       Guard to next domain: swing heel strikes

function domain = SingleSupportPointFoot(domain, stanceLeg, phaseType)

% Extract state variables
x = domain.States.x;

%% Relative degree 2 outputs -- joint positions (time-based)

%4 outputs -- order based on pinned model
if strcmp(stanceLeg,'Right')
    sl='Right'; nsl = 'Left'; 
    p_nsf = domain.getCartesianPosition(domain.ContactPoints.LeftSole); 
    p_sf  = domain.getCartesianPosition(domain.ContactPoints.RightSole);
else
    sl='Left'; nsl = 'Right';         
    p_nsf = domain.getCartesianPosition(domain.ContactPoints.RightSole); 
    p_sf  = domain.getCartesianPosition(domain.ContactPoints.LeftSole);
end

ya_2    = [x([sl, 'Knee']); x([sl, 'Hip']); x([nsl, 'Hip']), x([nsl, 'Knee'])];
y2_label = { [sl, 'Knee'],    [sl, 'Hip'],    [nsl, 'Hip'],    [nsl, 'Knee']};
    
%% Phase Variable - Time or State
switch phaseType
    
    case 'TimeBased'
    
    % use time as phase variable if time-based
    t = SymVariable('t');
    p = SymVariable('p',[2,1]);
    tau = (t-p(2))/(p(1)-p(2));
    
    case 'StateBased'
    
    % use linearized hip position as tau if phase-based
    hip_pos = domain.getCartesianPosition(domain.Joints(getJointIndices(domain,'RightHip')));
    phip = hip_pos(1) - p_sf(1);
    deltaphip = linearize(phip, x);
    
    
    %%% define tau(q)
    p = SymVariable('p',[2,1]);
    tau = (deltaphip - p(2))/(p(1)-p(2));
    
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

%%%%%%%%%%%%%%%%%%%%%%%%%% Contacts  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add Stance Heel Contact    
% domain = ContactFrames.PointFootCustom(domain, sl);

fric_coef = struct('mu',domain.fric_coef.mu);
    
if strcmp(stanceLeg,'Right')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% Contacts  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add Stance Heel Contact
    
    frame = ToContactFrame(domain.ContactPoints.RightSole, ...
                            'PlanarPointContactWithFriction');
    domain = addContact(domain, frame, fric_coef);
           
elseif strcmp(stanceLeg,'Left')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% Contacts  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Add Stance Heel Contact
    frame = ToContactFrame(domain.ContactPoints.LeftSole, ...
                            'PlanarPointContactWithFriction');
    domain = addContact(domain, frame, fric_coef);
          
else
    error("stanceLeg must be Left or Right");
end

%%%%%%%%%%%%%%%%%%%%%%%%% Guard Event  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Guard function assuming level ground
h_nsf = UnilateralConstraint(domain, p_nsf(3), 'nsf','x');
    
% add guard condition to domain
domain = addEvent(domain, h_nsf);

% IMPORTANT FOR TIME-BASED:
if strcmp(stanceLeg,'Right')
    domain.PreProcess = @Sim.NewStepPreProcess;
else
    domain.PreProcess = @Sim.MidStepPreProcess;
end
end
