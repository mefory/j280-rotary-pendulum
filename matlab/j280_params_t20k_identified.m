function P = j280_params_t20k_identified()
% T20K 实机辨识参数集（2026-11-02，C1+C2，见 config/plant_t20k_identified.json）
P = j280_params();
here = fileparts(mfilename('fullpath'));
profile = jsondecode(fileread(fullfile(here, '..', 'config', 'plant_t20k_identified.json')));
names = fieldnames(profile.parameters);
for index = 1:numel(names)
    P.(names{index}) = profile.parameters.(names{index});
end
end
