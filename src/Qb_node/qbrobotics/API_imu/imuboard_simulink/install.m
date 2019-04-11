%                                                             	    INSTALL
% This code install simulink library in your system and update
% 'path' variable with necessary paths 

clear all
clc
% Install library

cd library
make
cd ..

% LIBRARY UPDATE
display('[INFO] IMU board library should be installed after QB Move library');
answ = input('Do you wish to continue with the installation? ', 's');
if answ ~= 'y' && answ ~= 'Y'
    return    
end

lib = 'library/';

%retrieve current matlab version
ver = version('-release');
last_version_flag = false;
last_version = ver(1:end-1);
last_release=ver(end);

old_file = strcat('', lib);
old_file = strcat(old_file, 'vers/IMUboard_library_');
old_file = strcat(old_file, ver);

new_file = strcat('', lib);
new_file = strcat(new_file, 'IMUboard_library');

if ( (str2double(ver(1:end-1)) >= 2015) )
    old_file = strcat(old_file, '.slx');
    new_file = strcat(new_file, '.slx');
    
    if ~exist(old_file, 'file')
        last_version_flag = true;
        old_file = strcat('library/vers/IMUboard_library_', num2str(last_version));
        old_file = strcat(old_file, last_release);
        old_file = strcat(old_file, '.slx');
    end    
else
    old_file = strcat(old_file, '.mdl');
    new_file = strcat(new_file, '.mdl');
end

copyfile(old_file, new_file,'f');

old_file = '';
new_file = '';


% Add path to Set Path field
path(genpath(cd), path);

clc

flag_path = savepath;

while flag_path
    alternative_path = input('Chose an alternative path for pathdef.m file (without end slash): ', 's');
    
    if exist(alternative_path,'dir')
    
        alternative_path = strcat(alternative_path, '/pathdef.m');

        flag_path = savepath(alternative_path);

        if (flag_path)
            disp(['[ERROR] Path not correct. ' alaternative_path]);
        end
    else
        disp(['[ERROR] This is not a directory. ' alaternative_path]);
    end
end



if last_version_flag
    display('[WARNING] MATLAB version is not supported, last version available set.')
end

display('[INFO] IMU board library install successful');

clear all


