classdef behavior < handle
% Description: 
%    - model: planar_PF 
%    - state-based walking
% _________________________________________________________________________

    properties
        name;           % Name of this behavior
        isSymmetric;    % Flag true = symmetric behavior
        robotModel;     % The robot model associated
        vertices;       % Continuous domains
        edges;          % Discrete domains
        hybridSystem;   % Hybrid system for this behavior
        constraints;    % Optimization constraints
        phaseType;
        options;
    end
    
    methods
        function obj = init(obj, isSymmetric, phaseType, delay_coriolis, omit_coriolis)
            % Assign name
            obj.name = 'planar_PF';
            obj.isSymmetric = isSymmetric;
            
            % Load the namespace domains/edges
            import([obj.name, '.Vertices.*']); 
                        
            %%% Load in the model
            urdf_file = fullfile('urdf','five_link_walker.urdf');
            obj.robotModel = feval('FiveLinkModel', urdf_file);
            obj.robotModel.configureDynamics('DelayCoriolisSet', delay_coriolis,'OmitCoriolisSet',omit_coriolis);
            
            obj.phaseType = phaseType;
            
            %%% when some joint is actuated by springs
            % if strcmp(spring_conf, 'spring')
            %     obj.robotModel.appendDriftVector(obj.robotModel.fs_fun);
            % end
                                              
            %%% Load the controller
            controller  = IOFeedback('IO'); %Feedback linearizing controller
            
            %%% initialize the hybrid system
            obj.hybridSystem = HybridSystem(obj.name);
                        
            %% define hybrid systems
            
            if isSymmetric
                % --------------------------------------------------
                %  vertex1: RightStance                            |
                %  ->edge1: LeftImpactRelabel                      |
                % --------------------------------------------------
                obj.vertices.RightSS  = RightSS(obj.robotModel,phaseType);
                obj.edges.LeftImpactRelabel     = Edges.Impact(obj.vertices.RightSS, 'Right', true);
                
                obj.hybridSystem = addVertex(obj.hybridSystem , 'RightSS', ...
                                             'Domain', obj.vertices.RightSS, ...
                                             'Control', controller);
                
                obj.hybridSystem = addEdge(obj.hybridSystem , 'RightSS', 'RightSS');
                obj.hybridSystem = setEdgeProperties(obj.hybridSystem , ...
                                                     'RightSS', 'RightSS', ...
                                                     'Guard', obj.edges.LeftImpactRelabel);
                
            else
                error('Left and right are the same for 2D robot, symmetry is sufficient.');
                
            end
            
        end
    end
    
end
