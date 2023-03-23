function[]=constraintsFileFixedTop()
	global meshSize_finer meshSize_coarse
    
    meshSize=meshSize_finer;
	str="";
	for i=1:(meshSize+1)
		str=str+i;
		str=str+111;
		str=str+",";
	end
	Line=[str];	
	fid = fopen('../../vega_simulator/config/constraints_finer.bou', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("constraints_finer.bou has been created")
  
    meshSize=meshSize_coarse;
    str="";
	for i=1:(meshSize+1)
		str=str+i;
		str=str+111;
		str=str+",";
	end
	Line=[str];	
	fid = fopen('../../vega_simulator/config/constraints_coarse.bou', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("constraints_coarse.bou has been created")
end