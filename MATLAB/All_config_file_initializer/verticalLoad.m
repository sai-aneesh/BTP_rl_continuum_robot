function[]=verticalLoad(F)
    global meshSize_finer meshSize_coarse L_by_width
    meshSize=meshSize_finer;
    n=(meshSize+1)*(meshSize*L_by_width+1);
	str=(meshSize+1)+" ";
    for i=(n-meshSize):n
        if(i==(n-meshSize) || i==n)
            str=str+i+"010 ";
            force=F/(2*meshSize);
            str=str+force+" ";
        else
            str=str+i+"010 ";
            force=2*F/(2*meshSize);
            str=str+force+" ";
        end
    end
	Lines=[str];
    fid = fopen('../../vega_simulator/config/loads_finer.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("loads_finer.csv files has been created")
    
    meshSize=meshSize_coarse;
    n=(meshSize+1)*(meshSize*L_by_width+1);
	str=(meshSize+1)+" ";
    for i=(n-meshSize):n
        if(i==(n-meshSize) || i==n)
            str=str+i+"010 ";
            force=F/(2*meshSize);
            str=str+force+" ";
        else
            str=str+i+"010 ";
            force=2*F/(2*meshSize);
            str=str+force+" ";
        end
    end
	Lines=[str];
    fid = fopen('../../vega_simulator/config/loads_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("loads_coarse.csv files has been created")
end