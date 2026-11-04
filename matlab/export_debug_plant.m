function export_debug_plant()
% 导出上位机模拟器物理参数及方程校验向量，不复制一套手填参数。
here=fileparts(mfilename('fullpath'));P=j280_params();
bundle=struct('source','matlab/j280_params.m + j280_plant_rhs.m','parameters',P);
states=[0 pi 0 0; .3 .2 1 -2; -2 -1 -3 4; 1 2.5 5 -6];
torques=[0 .1 -.05 .2];derivatives=zeros(4,4);
for i=1:4,derivatives(i,:)=j280_plant_rhs(states(i,:)',torques(i),P)';end
bundle.states=states;bundle.torques=torques;bundle.derivatives=derivatives;
f=fopen(fullfile(here,'..','config','plant.json'),'w');
assert(f>=0);cleanup=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(bundle,PrettyPrint=true));
fprintf('Exported debug plant and four derivative reference vectors\n');
end
