function[]=momentAtEachSection(M)
    global meshSize_finer meshSize_coarse L_by_width nSection
    
    meshSize=meshSize_finer;
    numQuads=meshSize*meshSize*L_by_width;
    nNodesCols=meshSize+1;
    nRows=numQuads/meshSize;
    nRowsSection=nRows/nSection;
    if(mod(nRowsSection,1)~=0)
        error('the number of sections given cant divide beam into equal section check.')
    end
    str=nSection +" "; %first the number of sections are read in simulator to allocate space accordingly
    for i=1:nSection
        str=str+((i*nRowsSection)*nNodesCols-nNodesCols+1)+" ";
        str=str+((i*nRowsSection)*nNodesCols)+" ";
        str=str+M(i)+" ";
    end
	Lines=[str];
    fid = fopen('../../vega_simulator/config/moments_finer.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("moments_finer.bou files has been created")
    
    meshSize=meshSize_coarse;
    numQuads=meshSize*meshSize*L_by_width;
    nNodesCols=meshSize+1;
    nRows=numQuads/meshSize;
    nRowsSection=nRows/nSection;
    if(mod(nRowsSection,1)~=0)
        error('the number of sections given cant divide beam into equal section check.')
    end
    str=nSection +" "; %first the number of sections are read in simulator to allocate space accordingly
    for i=1:nSection
        str=str+((i*nRowsSection)*nNodesCols-nNodesCols+1)+" ";
        str=str+((i*nRowsSection)*nNodesCols)+" ";
        str=str+M(i)+" ";
    end
	Lines=[str];
    fid = fopen('../../vega_simulator/config/moments_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("moments_coarse.bou files has been created")
end