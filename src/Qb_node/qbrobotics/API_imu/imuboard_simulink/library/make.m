if exist(strcat(pwd, '/bin'), 'file')
    rmdir('bin', 's');
else
	mkdir(pwd, 'bin');
end

mex -outdir bin src/qbgetImuReadings_v1.cpp ../../qbAPI/src/qbmove_communications.cpp ../../qbAPI/src/imuboard_communications.cpp -Iinclude/
mex -outdir bin src/qbgetImuReadings_v2.cpp ../../qbAPI/src/qbmove_communications.cpp ../../qbAPI/src/imuboard_communications.cpp -Iinclude/