function[]=gravityFile()
	global gravity gravity_direction
	str=""+gravity+" "+gravity_direction;
	Line=[str];
	fid = fopen('../../vega_simulator/config/gravity.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("gravity.csv has been created")