%% Function: applyCost
%
% Description: Applies a desired cost function to continous domains of the
%   provided nlp. "CostType" should be a cell array of text strings
%   corresponding to the desired cost seen in the switch statement below.
%
% _________________________________________________________________________

function [ nlp ] = applyCost( behavior, nlp, CostType, weight, vars )

% Common variables
x = behavior.robotModel.States.x;
dx = behavior.robotModel.States.dx;
ddx = behavior.robotModel.States.ddx;


vhip = dx('BasePosX');
mg   = 9.81 * sum([behavior.robotModel.Links(:).Mass]); 

% Assign cost to each vertex
vertices = fields(behavior.vertices);
for i = 1:numel(vertices)
    phaseName = behavior.vertices.(vertices{i}).Name;
    phaseIndex = nlp.getPhaseIndex(phaseName);
    phase = nlp.Phase(phaseIndex);
    domain = phase.Plant;
    
    u  = domain.Inputs.Control.u;
    Be = domain.Gmap.Control.u;
%     p  = domain.Params.pposition;
%     a = SymVariable(tomatrix(domain.Params.aposition(:))); %a  = domain.Params.aposition;
    
    if contains(lower(domain.Name),'right')
        p_sf = domain.getCartesianPosition(domain.ContactPoints.RightSole);
        p_nsf = domain.getCartesianPosition(domain.ContactPoints.LeftSole);
    else
        p_sf = domain.getCartesianPosition(domain.ContactPoints.LeftSole);
        p_nsf = domain.getCartesianPosition(domain.ContactPoints.RightSole);
    end
    
    switch CostType{i}
        case 'mCOT'
            cot     = sqrt(sum((tovector(u)*(Be'*dx)).^2)).^2 / (mg * vhip);
            cot_fun = SymFunction(['cot_' phase.Name], cot, {u, dx});
            addRunningCost(nlp.Phase(phaseIndex), cot_fun, {'u', 'dx'});
        case 'impact'
            delta = 0.05;
            a = 100; % stepness of sigmoid (to approximate max function)
            if isfield(behavior.options,'Slope')
                slope = behavior.options.Slope;
                nsf_height = p_nsf(3) - (tan(slope)*p_nsf(1));
                soft_max = tovector((-nsf_height+delta)/(1+exp(a*(-delta+nsf_height))));
                impact_cost_fun = SymFunction(['impact_cost_',domain.Name],soft_max,{x});
            else
                error('not implemented yet');
            end
            addRunningCost(nlp.Phase(phaseIndex), impact_cost_fun, {'x'});
        case 'musCOT'
            sc = nlp.Phase(phaseIndex).Plant.Params.sc;
            cot     = sqrt(sum((tovector(u./sc)*(Be'*dx)).^2)).^2 / (mg * vhip);
            cot_fun = SymFunction(['musCoT_' phase.Name], cot, {u, dx,sc});
            addRunningCost(nlp.Phase(phaseIndex), cot_fun, {'u', 'dx','sc'});
            
        case 'TorqueSquare'
            u2 = weight.* tovector(norm(u).^2);
            u2_fun = SymFunction(['torque_', phase.Name], u2, {u});
            addRunningCost(nlp.Phase(phaseIndex), u2_fun, {'u'});                                   
            
        case 'GvecTorqueSquare'
            u2 = weight.* tovector(norm(nlp.Phase(phaseIndex).Plant.Gvec.Control.u).^2);
            u2_fun = SymFunction(['GvecTorque_', phase.Name], u2, {u});
            addRunningCost(nlp.Phase(phaseIndex), u2_fun, {'u'});    
            
        case 'BaseMovement'
            if nargin < 5
                vars = [0;0];
            end
            
            if strcmp(behavior.robotModel.Joints(6).Name, 'BaseRotZ')
                qbIndices = 1:6;
                auxdata = [vars; zeros(4,1)];
            else
                qbIndices = 1:3;
                auxdata = [vars;0];
            end
            
            vd = SymVariable('vd',[length(auxdata),1]);
            
            baseMov = weight.* tovector(sum((vd - dx(qbIndices)).^2));
            baseMovFun = SymFunction(['BaseMovement_', phase.Name], baseMov, {dx}, {vd});
            addRunningCost(nlp.Phase(phaseIndex), baseMovFun, {'dx'}, {auxdata});
            
        case 'NSFMovement'
            if isempty(strfind(domain.Name, 'RightSS'))
                p_nsf = getCartesianPosition(domain, domain.RefPoints.RightSoleJoint);
            else
                p_nsf = getCartesianPosition(domain, domain.RefPoints.LeftSoleJoint);
            end
            v_nsf = jacobian(p_nsf, x) * dx;
            
            aux_data = [vars;0];
            vd = SymVariable('vd',[length(aux_data),1]);
            nsf_vel_norm = tovector(sum((vd*2 - v_nsf).^2));
            nsf_vel_norm_Fun = SymFunction(['NSFMovement_', phase.Name], nsf_vel_norm, {x, dx}, {vd});
            addRunningCost(nlp.Phase(phaseIndex), nsf_vel_norm_Fun, {'x','dx'}, {aux_data});
            
        case 'q_ddx'
             accel = sum(weight*tovector(ddx([5,6]).^2));
%             accel = weight*tovector(norm(ddx).^2);
            ddx_Fun = SymFunction(['jointAnkleAcceleration_',domain.Name], accel, {ddx});
            addRunningCost(nlp.Phase(phaseIndex), ddx_Fun, {'ddx'});
           
        case 'HipMatch'
            weight1 = weight;
            switch phaseName
                case'RightSS2'
                    alpha_des = behavior.ref_traj.RS;
                    alpha_p = behavior.vertices.(vertices{i}).VirtualConstraints.position.OutputParams;
                    alpha_offset = behavior.vertices.(vertices{i}).Params.alphaOffset;
                    ref_diff_p = weight1.*bezier_integral(bezier_square(alpha_p(1,:)+alpha_des.RHIP'-alpha_offset(1)))+...
                                     weight1.*bezier_integral(bezier_square(alpha_p(4,:)+alpha_des.LHIP'-alpha_offset(4)));
                    ref_diff = ref_diff_p(1);
                    ref_diff_fun = SymFunction(['ref_diff_hip',phaseName], ref_diff, {tovector(alpha_p), tovector(alpha_offset)});
                    addNodeCost(nlp.Phase(phaseIndex), ref_diff_fun, {'aposition', 'alphaOffset'},'last');

                
                case 'LeftSS2'
                    alpha_des = behavior.ref_traj.LS;
                    alpha_p = behavior.vertices.(vertices{i}).VirtualConstraints.position.OutputParams;
                    alpha_offset = behavior.vertices.(vertices{i}).Params.alphaOffset;
                    ref_diff_p = weight1.*bezier_integral(bezier_square(alpha_p(1,:)+alpha_des.LHIP'-alpha_offset(1)))+...
                                 weight1.*bezier_integral(bezier_square(alpha_p(4,:)+alpha_des.RHIP'-alpha_offset(4)));

                    ref_diff = ref_diff_p(1);
                    ref_diff_fun = SymFunction(['ref_diff_hip',phaseName], ref_diff, {tovector(alpha_p), tovector(alpha_offset)});
                    addNodeCost(nlp.Phase(phaseIndex), ref_diff_fun, {'aposition', 'alphaOffset'},'last');

            end
        case 'KneeMatch'
            weight1 = weight;
            switch phaseName
                case'RightSS2'
                    alpha_des = behavior.ref_traj.RS;
                    alpha_p = behavior.vertices.(vertices{i}).VirtualConstraints.position.OutputParams;
                    alpha_offset = behavior.vertices.(vertices{i}).Params.alphaOffset;
                    ref_diff_p = weight1.*bezier_integral(bezier_square(alpha_p(2,:)+alpha_des.RKNE'-alpha_offset(2)))+...
                                     weight1.*bezier_integral(bezier_square(alpha_p(5,:)+alpha_des.LKNE'-alpha_offset(5)));
                    ref_diff = ref_diff_p(1);
                    ref_diff_fun = SymFunction(['ref_diff_knee',phaseName], ref_diff, {tovector(alpha_p), tovector(alpha_offset)});
                    addNodeCost(nlp.Phase(phaseIndex), ref_diff_fun, {'aposition', 'alphaOffset'},'last');

                
                case 'LeftSS2'
                    alpha_des = behavior.ref_traj.LS;
                    alpha_p = behavior.vertices.(vertices{i}).VirtualConstraints.position.OutputParams;
                    alpha_offset = behavior.vertices.(vertices{i}).Params.alphaOffset;
                    ref_diff_p = weight1.*bezier_integral(bezier_square(alpha_p(2,:)+alpha_des.LKNE'-alpha_offset(2)))+...
                                 weight1.*bezier_integral(bezier_square(alpha_p(5,:)+alpha_des.RKNE'-alpha_offset(5)));

                    ref_diff = ref_diff_p(1);
                    ref_diff_fun = SymFunction(['ref_diff_knee',phaseName], ref_diff, {tovector(alpha_p), tovector(alpha_offset)});
                    addNodeCost(nlp.Phase(phaseIndex), ref_diff_fun, {'aposition', 'alphaOffset'},'last');

            end
        case 'AnkleMatch'
            weight1 = weight;
            switch phaseName
                case'RightSS2'
                    alpha_des = behavior.ref_traj.RS;
                    alpha_p = behavior.vertices.(vertices{i}).VirtualConstraints.position.OutputParams;
                    alpha_offset = behavior.vertices.(vertices{i}).Params.alphaOffset;
                    ref_diff_p = weight1.*bezier_integral(bezier_square(alpha_p(3,:)+alpha_des.RANK'-alpha_offset(3)))+...
                                 weight1.*bezier_integral(bezier_square(alpha_p(6,:)+alpha_des.LANK'-alpha_offset(6)));
                    ref_diff = ref_diff_p(1);
                    ref_diff_fun = SymFunction(['ref_diff_ankle',phaseName], ref_diff, {tovector(alpha_p), tovector(alpha_offset)});
                    addNodeCost(nlp.Phase(phaseIndex), ref_diff_fun, {'aposition', 'alphaOffset'},'last');

                
                case 'LeftSS2'
                    alpha_des = behavior.ref_traj.LS;
                    alpha_p = behavior.vertices.(vertices{i}).VirtualConstraints.position.OutputParams;
                    alpha_offset = behavior.vertices.(vertices{i}).Params.alphaOffset;
                    ref_diff_p = weight1.*bezier_integral(bezier_square(alpha_p(3,:)+alpha_des.LANK'-alpha_offset(3)))+...
                                 weight1.*bezier_integral(bezier_square(alpha_p(6,:)+alpha_des.RANK'-alpha_offset(6)));

                    ref_diff = ref_diff_p(1);
                    ref_diff_fun = SymFunction(['ref_diff_ankle',phaseName], ref_diff, {tovector(alpha_p), tovector(alpha_offset)});
                    addNodeCost(nlp.Phase(phaseIndex), ref_diff_fun, {'aposition', 'alphaOffset'},'last');

            end
    end
end

end