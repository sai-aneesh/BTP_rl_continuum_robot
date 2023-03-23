function[]=TwoRigidBarAlongWidthStiffness()
	global meshSize_finer meshSize_coarse L_by_width youngsModulus depth StifferYoungModulus
    
    meshSize=meshSize_finer;
	Stiffness=3*youngsModulus*depth/8;
    rigidStiffness=3*StifferYoungModulus*depth/8;
	numQuads=meshSize*meshSize*L_by_width;
	str="";
    nRows=numQuads/meshSize;
    for i=1:numQuads
		if(i>=(ceil(nRows*0.5)*meshSize+1) && i<=(ceil(nRows*0.5)*meshSize+4))
            str=str+rigidStiffness+" ";
        elseif (i>=(numQuads-3) && i<=(numQuads))
            str=str+rigidStiffness+" ";
        else
            str=str+Stiffness+" ";
        end
    end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k_finer.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k_finer.csv has been created")
    
    meshSize=meshSize_coarse;
    Stiffness=3*youngsModulus*depth/8;
    rigidStiffness=3*StifferYoungModulus*depth/8;
	numQuads=meshSize*meshSize*L_by_width;
	str="";
    nRows=numQuads/meshSize;
    for i=1:numQuads
		if(i>=(ceil(nRows*0.5)*meshSize+1) && i<=(ceil(nRows*0.5)*meshSize+4))
            str=str+rigidStiffness+" ";
        elseif (i>=(numQuads-3) && i<=(numQuads))
            str=str+rigidStiffness+" ";
        else
            str=str+Stiffness+" ";
        end
    end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k_coarse.csv has been created")
end