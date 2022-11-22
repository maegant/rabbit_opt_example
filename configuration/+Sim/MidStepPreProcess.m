function params = MidStepPreProcess(sys, t, x, controller, params) %#ok<INUSL>
    
       y = struct2array(sys.VirtualConstraints);
    
    for i=1:numel(y)
        if strcmp(y(i).PhaseType,'TimeBased')
            params.(y(i).PhaseParamName) = controller.Param.time_start + params.(y(i).PhaseParamName);
        elseif strcmp(y(i).PhaseType, 'StateBased') && i == numel(y)
            nx = length(x)/2;
            tau = calcPhaseVariable(y(end), t, x(1:nx), x(nx+1:end), params.(y(i).PhaseParamName));
            delta = tau{1}*(params.(y(i).PhaseParamName)(1) - params.(y(i).PhaseParamName)(2)) ...
                    +  params.(y(i).PhaseParamName)(2);
            params.(y(i).PhaseParamName)(2) =  delta;
         
%             params.(y(i).PhaseParamName)(1) =  params.(y(i).PhaseParamName)(1) - params.(y(i).PhaseParamName)(2) + delta;
%             params.(y(i).PhaseParamName) = controller.Param.time_start + params.(y(i).PhaseParamName);
        end
    end
end