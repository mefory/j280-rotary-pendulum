function P = j280_params_de10_identified()
P = j280_params();
here = fileparts(mfilename('fullpath'));
profile = jsondecode(fileread(fullfile(here, '..', 'config', 'plant_de10_identified.json')));
names = fieldnames(profile.parameters);
for index = 1:numel(names)
    P.(names{index}) = profile.parameters.(names{index});
end
end
