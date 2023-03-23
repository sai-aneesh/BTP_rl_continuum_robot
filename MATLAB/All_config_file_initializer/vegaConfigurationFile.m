function[]=vegaConfigurationFile()
   global meshSize_finer meshSize_coarse Beam_width force_ramp time_for_full_load impulse_force timed_force
   meshSize=meshSize_finer;
	str="";
    str=str+(meshSize+1)+" ";
    str=str+Beam_width+ " ";
    str=str+force_ramp+" ";
    str=str+time_for_full_load+" ";
    str=str+impulse_force+" ";
    str=str+timed_force+" ";
    Lines=[str];
	fid = fopen('../../vega_simulator/config/vegaConfigurations_finer.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("vegaConfigurations_finer.csv files has been created")
    
    meshSize=meshSize_coarse;
    str="";
    str=str+(meshSize+1)+" ";
    str=str+Beam_width+ " ";
    str=str+force_ramp+" ";
    str=str+time_for_full_load+" ";
    str=str+impulse_force+" ";
    str=str+timed_force+" ";
    Lines=[str];
	fid = fopen('../../vega_simulator/config/vegaConfigurations_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("vegaConfigurations_coarse.csv files has been created")
end