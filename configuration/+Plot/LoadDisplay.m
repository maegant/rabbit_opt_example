function robot_disp = LoadDisplay(behavior, varargin)
    robot = behavior.robotModel;
    skipExport = true;
    
    root_path = pwd;
    export_path = fullfile(root_path, 'export', 'model', behavior.robotModel.Name, 'animator');
    if ~exist(export_path,'dir')
        mkdir(export_path);
        skipExport = false;
    end
    addpath(export_path);
    
    if nargin > 2
       numSims = varargin(1);
       currentSim = varargin(2);
    else
       numSims = 1; currentSim = 1;
    end
    
    if nargin > 5
        options = varargin(3:end);
    else        
        options = {'UseExported', true, 'ExportPath', export_path, 'SkipExporting', skipExport};
    end

    f = figure();clf;
%     set(gca,'xlim',[0,10])
    if currentSim == 1
        clf;
    end
    robot_disp = frost.Animator.Display(f,robot, options{:});

    
    ref_item{1} = robot.ContactPoints.LeftSole;
    ref_item{2} = robot.ContactPoints.RightSole;
    for i = 1:2
        item =  frost.Animator.Sphere(robot_disp.axs, robot, ref_item{i}, ref_item{i}.Name, options{:});
        robot_disp.addItem(item);
    end
    
    % If seven link model (with feet), add toe and heel contacts
    if robot.numState == 9
        ref_item{3} = robot.ContactPoints.LeftToe;
        ref_item{4} = robot.ContactPoints.LeftHeel;
        ref_item{5} = robot.ContactPoints.RightToe;
        ref_item{6} = robot.ContactPoints.RightHeel;
        for i = 3:6
            item =  frost.Animator.Sphere(robot_disp.axs, robot, ref_item{i}, ref_item{i}.Name, options{:});
            robot_disp.addItem(item);
        end
    end
    
    % Add link between knee and foot
    if robot.numState == 7
        parentName = {'LeftKnee','RightKnee'};
        childName = {'LeftSole','RightSole'};
        
        for i = 1:2
        name = ['Link_', parentName{i}, '_to_', childName{i}];
        frame = robot.ContactPoints.(childName{i});
        
        item = frost.Animator.Cylinder(robot_disp.axs, robot, frame, -frame.Offset, name, varargin{:});
        robot_disp.addItem(item);
        end   
           
    end
    
    % Add Torso Link
    torso_frame = robot.OtherPoints.Torso;
    torso_link = frost.Animator.Cylinder(robot_disp.axs, robot, torso_frame, -torso_frame.Offset, 'Link_Torso', varargin{:});
    robot_disp.addItem(torso_link);
    torso_end =  frost.Animator.Sphere(robot_disp.axs, robot, torso_frame, torso_frame.Name, options{:});
    robot_disp.addItem(torso_end);
    
    title(sprintf('Simulation %i of %i',currentSim, numSims));
    set(robot_disp.axs,'XLim',[-0.5,10]);
    view(robot_disp.axs,[0,0]);
    robot_disp.update(zeros(robot.numState,1));
end