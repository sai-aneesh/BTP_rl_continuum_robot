function[]=noLoad()
	Lines=["0"];
	fid = fopen('../../vega_simulator/config/loads_finer.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("loads_finer.bou files has been created")
    
    Lines=["0"];
	fid = fopen('../../vega_simulator/config/loads_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("loads_coarse.bou files has been created")
end