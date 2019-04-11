%-------------------------------------------------------------------------------
% Mask drawing routine
% Sets ports label and parameters visibility according to choices made in the
% dialog box of the sfunction qbot_ISS
%-------------------------------------------------------------------------------
%==========       UNLOCK LIBRARY IF IT CALLED THE SCRIPT (done to avoid warning)
if strcmpi( gcs, 'qb_library_ISS' )
    set_param('qb_library_ISS', 'lock', 'off');
end
%==================================================       BLOCK BACKGROUND IMAGE
% image( imread('qbot.jpg') );
%=======================================================     TEMPORARY VARIABLES
tmp_MaskEnables = get_param( gcb, 'MaskEnables' );              % actual config
tmp_MaskVisibilities = get_param( gcb, 'MaskVisibilities' );   % actual config
%====================================================================     INPUTS
port_label( 'input', 1, 'handle' );

%==================================================================      OUTPUTS

% IMU configuration
% 4 parameters for each imu: ID - Acc flag - Gyro flag - Mag flag
for i= 10 : 77
    tmp_MaskVisibilities{i} = 'off';
end

maskStr = get_param(gcb,'N_IMU');

n_imu = str2double(maskStr);

for i = 1 : n_imu
    tmp_MaskVisibilities{10 + 4*(i-1)} = 'on';
    tmp_MaskVisibilities{10 + 4*(i-1) + 1} = 'on';
    tmp_MaskVisibilities{10 + 4*(i-1) + 2} = 'on';
    tmp_MaskVisibilities{10 + 4*(i-1) + 3} = 'on';
end
set_param( gcb, 'MaskVisibilities', tmp_MaskVisibilities );


% Set output ports labels
for i=1:n_imu
    ss = sprintf('imu_%d', i-1); 
    port_label( 'output', i, ss );
end
