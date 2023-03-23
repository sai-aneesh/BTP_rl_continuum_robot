function[]=sameStiffness()
	global meshSize_finer meshSize_coarse L_by_width youngsModulus depth
    
    meshSize=meshSize_finer;
	Stiffness=3*youngsModulus*depth/8;
	numQuads=meshSize*meshSize*L_by_width;
	str="";
    str=str+numQuads+" ";
	for i=1:numQuads
		str=str+Stiffness+" ";
	end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k_finer.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k_finer.csv has been created")
    
    meshSize=meshSize_coarse;
	Stiffness=3*youngsModulus*depth/8;
	numQuads=meshSize*meshSize*L_by_width;
	str="";
    str=str+ numQuads +" ";
	for i=1:numQuads
		str=str+Stiffness+" ";
	end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k_coarse.csv has been created")
end