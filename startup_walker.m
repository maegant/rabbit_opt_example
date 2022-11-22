function behavior = startup_walker(do_compile)
% Description:
%   Select behaviorName based on desired model/behavior
%       1) planar_PF: uses five link model for point-foot walking
%       2) planar_FF: uses seven link model for flat-foot walking
%       3) planar_full: uses seven link model for multi-contact walking
% 
%   Also select phaseType as either TimeBased or PhaseBased (only TimeBased
%       tested as of right now)
%
%   author: Maegan Tucker (mtucker@caltech.edu)
% _________________________________________________________________________

addpath(genpath('configuration'));
addpath(genpath('utils'));
addpath(genpath('model'));
addpath(genpath('simulator'));

%%  Run the associated behavior constructor %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
do_export_model = do_compile(1);
do_export_behavior = do_compile(2);

behaviorName = 'planar_FF'; 
phaseType = 'StateBased';
is_symmetric = true;

delay_coriolis = false;
omit_coriolis = false;

behavior = loadBehavior(behaviorName, is_symmetric, phaseType, ...
                delay_coriolis, omit_coriolis, do_export_behavior, do_export_model);