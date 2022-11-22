function guard = Impact(tar, stanceFoot, isRelabel)
    
    % choose proper type
    event_name = 'nsf';

    % set impact
    if strcmp(stanceFoot,'Right')
        
        if isRelabel
            guard = RigidImpact('LeftImpactRelabel',tar,event_name);
        else
            guard = RigidImpact('LeftImpact',tar,event_name);
        end
        
    elseif strcmp(stanceFoot,'Left')
        
        if isRelabel
            guard = RigidImpact('RightImpactRelabel',tar,event_name);
        else
            guard = RigidImpact('RightImpact',tar,event_name);
        end
        
    else
        error('something is wrong');
    end
    
    % Relabeling Matrix
    if isRelabel
        
       if tar.numState == 7
            guard.R = guard.R(:,[1:3,6:7,4:5]);
        else
            guard.R = guard.R(:,[1:3,7:9,4:6]);
        end
        
    end
    
    % set the impact constraint
    % we will compute the impact map every time you add an impact
    % constraints, so it would be helpful to load expressions directly
    guard.addImpactConstraint(struct2array(tar.HolonomicConstraints));

end