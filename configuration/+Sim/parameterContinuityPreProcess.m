function params = parameterContinuityPreProcess(sys, t, x, controller, params) %#ok<INUSL>
    
    y = struct2array(sys.VirtualConstraints);
    
    for i=1:numel(y)
        if strcmp(y(i).PhaseType,'TimeBased')
            param_diff = params.(y(i).PhaseParamName)(1) - params.(y(i).PhaseParamName)(2); % time span for this domain
            params.(y(i).PhaseParamName)(2) = t; % initial time
            params.(y(i).PhaseParamName)(1) = t + param_diff; % final time
        end
    end
end