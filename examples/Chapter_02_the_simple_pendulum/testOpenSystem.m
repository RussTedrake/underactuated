function testOpenSystem

cd(fullfile(getDrakePath,'examples','Pendulum'));
open_system('constantTorqueDemo');
% hit play to start the simulation, and click on the boxes labeled as
% 'slider' to control the torque and damping.
% if you get an error about not keeping up with real time, then
% double-click on the realtime block and reduce the realtime factor
% (e.g. to .5).

