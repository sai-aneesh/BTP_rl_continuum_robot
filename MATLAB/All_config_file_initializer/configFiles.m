function[]=configFiles()
	global density youngsModulus depth gravity MassDamping StiffnesDamping initialPose
	%starting few lines for main config file
	Lines=["*solver";"implicitNewmark";"";"*massSpringSystemObjConfigFilename";"beam3_vox_finer.massspring";"";"*renderingMeshFilename";"beam_finer.obj";""]; %staring few lines
	strM=""+MassDamping;
	strK=""+StiffnesDamping;
	Lines=[Lines;"*dampingMassCoef";strM;"";"*dampingStiffnessCoef";strK;""];
	Lines=[Lines;"*fixedVerticesFilename";"constraints_finer.bou";"";"*forceLoadsFilename";"loads_finer.csv";"";"*momentRigidSectionLoadsFilename";"moments_finer.csv";"";"*vegaConfigurationsFilename";"vegaConfigurations_finer.csv";""];
	if(initialPose)
		Lines=[Lines;"*initialPositionFilename";"initial_finer.bou";""];
	end
		
	%writing the beam3_vox_massspring.config
	fid = fopen('../../vega_simulator/config/beam3_vox_massspring_finer.config', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("beam3_vox_massspring_finer.config file has been created")

	% starting few lines for the .masspring file
	Lines=["*massSpringMeshFilename";"beam_finer.obj";"";"*surfaceDensity"];
	surfDensity=""+density*depth;
	Stiffness=""+3*depth*youngsModulus/8;
	gravity_=""+gravity;
	Lines=[Lines;surfDensity;"";"*tensileStiffness";Stiffness;"";"*shearStiffness";Stiffness;"";"*bendStiffness";"0.0";"";"*damping";"0.0";"";"*addgravity";gravity_];

	%writing the beam3_vox.massspring
	fid = fopen('../../vega_simulator/config/beam3_vox_finer.massspring', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("beam3_vox_finer.massspring file has been created")
    
    %% starting few lines for main config file
	Lines=["*solver";"implicitNewmark";"";"*massSpringSystemObjConfigFilename";"beam3_vox_coarse.massspring";"";"*renderingMeshFilename";"beam_coarse.obj";""]; %staring few lines
	strM=""+MassDamping;
	strK=""+StiffnesDamping;
	Lines=[Lines;"*dampingMassCoef";strM;"";"*dampingStiffnessCoef";strK;""];
	Lines=[Lines;"*fixedVerticesFilename";"constraints_coarse.bou";"";"*forceLoadsFilename";"loads_coarse.csv";"";"*momentRigidSectionLoadsFilename";"moments_coarse.csv";"";"*vegaConfigurationsFilename";"vegaConfigurations_coarse.csv";""];
	if(initialPose)
		Lines=[Lines;"*initialPositionFilename";"initial.bou";""];
	end
		
	%writing the beam3_vox_massspring.config
	fid = fopen('../../vega_simulator/config/beam3_vox_massspring_coarse.config', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("beam3_vox_massspring_coarse.config file has been created")

	%starting few lines for the .masspring file
	Lines=["*massSpringMeshFilename";"beam_coarse.obj";"";"*surfaceDensity"];
	surfDensity=""+density*depth;
	Stiffness=""+3*depth*youngsModulus/8;
	gravity_=""+gravity;
	Lines=[Lines;surfDensity;"";"*tensileStiffness";Stiffness;"";"*shearStiffness";Stiffness;"";"*bendStiffness";"0.0";"";"*damping";"0.0";"";"*addgravity";gravity_];

	%writing the beam3_vox.massspring
	fid = fopen('../../vega_simulator/config/beam3_vox_coarse.massspring', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("beam3_vox_coarse.massspring file has been created")
end