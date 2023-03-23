function[]=nRigidBarAlongWidthStiffness()
    %this function will create stifness file such that there will be n
    %rigid plate including the tip
    global meshSize_finer meshSize_coarse L_by_width youngsModulus depth StifferYoungModulus nSection
	Stiffness=3*youngsModulus*depth/8;
    rigidStiffness=3*StifferYoungModulus*depth/8;
    
	meshSize=meshSize_finer;
    numQuads=meshSize*meshSize*L_by_width;
    nRows=numQuads/meshSize;
    nRowsSection=nRows/nSection;
    if(mod(nRowsSection,1)~=0)
        error('the number of sections given cant divide beam into equal section check.')
    end
    StiffnessArray=zeros(numQuads,1)+Stiffness;
	
    str="";
    str=str+numQuads+" ";
    for i=1:nSection
        StiffnessArray((i*nRowsSection*meshSize-2*meshSize+1):i*nRowsSection*meshSize)=rigidStiffness;
%         (i*nRowsSection*meshSize-meshSize+1):i*nRowsSection*meshSize
    end
    for i=1:numQuads
            str=str+StiffnessArray(i)+" ";
    end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k_finer.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k_finer.csv has been created")
    
    meshSize=meshSize_coarse;
    numQuads=meshSize*meshSize*L_by_width;
    nRows=numQuads/meshSize;
    nRowsSection=nRows/nSection;
    if(mod(nRowsSection,1)~=0)
        error('the number of sections given cant divide beam into equal section check.')
    end
    StiffnessArray=zeros(numQuads,1)+Stiffness;
    str="";
    str=str+numQuads+" ";
    for i=1:nSection
        StiffnessArray((i*nRowsSection*meshSize-2*meshSize+1):i*nRowsSection*meshSize)=rigidStiffness;
%         (i*nRowsSection*meshSize-meshSize+1):i*nRowsSection*meshSize
    end
    for i=1:numQuads
            str=str+StiffnessArray(i)+" ";
    end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k_coarse.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k_coarse.csv has been created")
end