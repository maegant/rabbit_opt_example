%% Function: enforceEdgeParameterContinuity
%
% Description: Formulates parameter continuity conditions between adjacent
%   domains for a provided edge.
%
% Author: Jenna Reher, jreher@caltech
% ________________________________________

function [ ] = enforceEdgeParameterContinuity( nlp, src, tar )

% Pull the NLP plant
plant = nlp.Plant;

%% Add avelocity continuity constraints
if isfield(src.Plant.VirtualConstraints, 'velocity') && isfield(tar.Plant.VirtualConstraints, 'velocity')
    sc_s = SymVariable('avs', [1,1]);
    sc_t = SymVariable('avt', [1,1]);
    sc_cont = SymFunction(['avelocityCont', plant.Name], sc_s - sc_t, {sc_s, sc_t});

    sc_cstr = NlpFunction('Name','avelocityCont',...
        'Dimension',1,...
        'lb', 0,...
        'ub', 0,...
        'Type','Linear',...
        'SymFun', sc_cont,...
        'DepVariables',[src.OptVarTable.avelocity(end); tar.OptVarTable.avelocity(1)]);
    nlp.addConstraint('avelocityCont', 'first', sc_cstr); 
end
%%
if isfield(src.Plant.Params, 'sc') && isfield(tar.Plant.Params, 'sc')
    sz = length(src.Plant.Params.sc);
    sc_s = SymVariable('scs', [sz,1]);
    sc_t = SymVariable('sct', [sz,1]);
    sc_cont = SymFunction(['sc_Cont', plant.Name], sc_s - sc_t, {sc_s, sc_t});

    sc_cstr = NlpFunction('Name','sc_Cont',...
        'Dimension',sz,...
        'lb', 0,...
        'ub', 0,...
        'Type','Linear',...
        'SymFun', sc_cont,...
        'DepVariables',[src.OptVarTable.sc(end); tar.OptVarTable.sc(1)]);
    nlp.addConstraint('sc_Cont', 'first', sc_cstr); 
end

%% Add pposition continutity constraints (pposition_DS = pposition_SS)
if isempty(strfind(nlp.Name, 'Impact')) % if domains are connected without an impact, make continuous
    pp_s = SymVariable('pps', [2,1]);
    pp_t = SymVariable('ppt', [2,1]);

    pp_eq = SymFunction(['ppositionEquality', plant.Name], pp_s - pp_t, {pp_s, pp_t});
    pp_cstr = NlpFunction('Name','ppositionEquality',...
        'Dimension', 2,...
        'lb', zeros(2,1),...
        'ub', zeros(2,1),...
        'Type', 'Linear',...
        'SymFun', pp_eq,...
        'DepVariables',[src.OptVarTable.pposition(end); tar.OptVarTable.pposition(1)]);
    nlp.addConstraint('ppositionEquality', 'first', pp_cstr); 
end

%% Make Bezier polynomials of matching names equal through domain
% It is HIGHLY suggested that all y in DS are in SS
if isempty(strfind(nlp.Name, 'Impact')) % if domains are connected without an impact, make continuous
    assert(src.Plant.VirtualConstraints.position.PolyDegree == ...
           tar.Plant.VirtualConstraints.position.PolyDegree, ...
           'The polynomial degree for target and source domains MUST be equal!');

    M     = src.Plant.VirtualConstraints.position.PolyDegree;
    ny_ds = src.Plant.VirtualConstraints.position.Dimension;
    ny_ss = tar.Plant.VirtualConstraints.position.Dimension;

    y_name_ds = src.Plant.VirtualConstraints.position.OutputLabel;
    y_name_ss = tar.Plant.VirtualConstraints.position.OutputLabel;

    ap_s = SymVariable('aps', [ny_ds, M+1]);
    ap_t = SymVariable('apt', [ny_ss, M+1]);

    for i = 1:ny_ds
        % Check for match and get its index in the SS domain
        yi(i) = find(strcmp(y_name_ss, y_name_ds{i}));
        if all(isempty(yi))
            continue; % No match if you got here
        end
    end

    aDiff = SymExpression(zeros(length(yi), M+1));
    for j = 1:numel(yi)
        aDiff(j,:) = ap_s(j,:) - ap_t(yi(j),:);
    end
    aDiff = flatten(aDiff(:))';

    na = length(aDiff);
    aDiff_eq = SymFunction(['apositionEquality', plant.Name], aDiff, {SymVariable(flatten(ap_s(:))'), ...
                                                        SymVariable(flatten(ap_t(:))')});
    ap_cstr = NlpFunction('Name','apositionEquality',...
        'Dimension', na,...
        'lb', zeros(na,1),...
        'ub', zeros(na,1),...
        'Type', 'Linear',...
        'SymFun', aDiff_eq,...
        'DepVariables',[src.OptVarTable.aposition(end); tar.OptVarTable.aposition(1)]);
    nlp.addConstraint('apositionEquality', 'first', ap_cstr); 
end

end