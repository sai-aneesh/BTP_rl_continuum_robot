function[]=matlabConfigforReadingData()
    global meshSize_finer meshSize_coarse L_by_width Beam_width
    
    meshSize=meshSize_finer;
    n=(meshSize+1)*(meshSize*L_by_width+1);
	Lines=[];
    str="";
    str=str+n+",";
    str=str+(meshSize+1)+",";
    str=str+Beam_width;
    Lines=[str];
	fid = fopen('../analysingVegaData/config_finer.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("matlab_coarse config file has been created")
    
    meshSize=meshSize_coarse;
    n=(meshSize+1)*(meshSize*L_by_width+1);
	Lines=[];
    str="";
    str=str+n+",";
    str=str+(meshSize+1)+",";
    str=str+Beam_width;
    Lines=[str];
	fid = fopen('../analysingVegaData/config_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("matlab_coarse config file has been created")
end