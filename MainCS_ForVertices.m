%% Main Script for Coordinate System Toolbox
function [origin_point, direction_point] = MainCS_ForVertices(vertices, bone_indx, side_indx, predicted, plot_different)
% clear, clc, close all
close all, clc
% This main code only requires the users bone model input. Select the
% folder where the file is and then select the bone model(s) you wish the
% apply a coordinate system to.

% Ensure that there are no spaces in the folder name, consider replacing 
% spaces with underscores (_).

% Currently, this code works for all bones from the tibia and fibula
% through the metatarsals. It has an option for multiple coordinate
% systems for the talus and calcaneus. I also can place the origin of the
% coordinate system at a joint surface.

% While it's not neccessary, naming your file with the laterality (_L_ or
% _Left_ etc.) and the name of the bone (_Calcaneus) will speed up the
% process. I recommend a file name similar to this for ease:
% group_#_bone_laterality.stl (ex. ABC_01_Tibia_Right.stl)

% Determine the files in the folder selected
%RBF fit

% temp = strfind(FolderPathName,'\');
% FolderName = FolderPathName(temp(end)+1:end); % Extracts the folder name selected

%% Load all files into list
% temp = struct2cell(files);
% list_files = temp(1,:);
% % Filter list_files for .ply files (case-insensitive)
% is_ply = endsWith(lower(list_files), '.ply');
% all_files = list_files(is_ply);
% all_files = all_files(:)'; % row vector to match expected shape


% Caches: key = bone_indx (int), value = selections for that bone type
cs_cache    = containers.Map('KeyType','int32','ValueType','any');  % stores struct with fields: bone_coord, cs_string
joint_cache = containers.Map('KeyType','int32','ValueType','any');  % stores scalar joint_indx


% Lists for detemining bone and side
list_bone = {'Talus', 'Calcaneus', 'Navicular', 'Cuboid', 'Medial_Cuneiform','Intermediate_Cuneiform',...
    'Lateral_Cuneiform','Metatarsal1','Metatarsal2','Metatarsal3','Metatarsal4','Metatarsal5',...
    'Tibia','Fibula'};
% bone_indx = 14; %fibula
% bone_indx = 13; %tibia
% % bone_indx = 1; %talus
% % bone_indx = 2; %calcaneus
% bone_indx = 5;% Tarsals_L_1, ie medial cuneiform
% % bone_indx = 6; %tarsals_L_2, intermediate cuneiform
% % bone_indx = 7; %Tarsals_L_3, lateral cuneiform
% % bone_indx = 4; %Tarsals_L_4, cuboid
% % bone_indx = 3; %Tarsals_L_5, navicular
% bone_indx = 8; %Metatarsals_L_1
% bone_indx = 9; %Metatarsals_L_2
% bone_indx = 10; %Metatarsals_L_3
% bone_indx = 11; %Metatarsals_L_4
% bone_indx = 12; %Metatarsals_L_5

list_bone2 = {'Talus', 'Calcaneus', 'Navicular', 'Cuboid', 'Med_Cuneiform','Int_Cuneiform',...
    'Lat_Cuneiform','First_Metatarsal','Second_Metatarsal','Third_Metatarsal','Fourth_Metatarsal','Fifth_Metatarsal',...
    'Tibia','Fibula'};
list_side_folder = {'Right','_R.','_R_','Left','_L.','_L_'};
list_side = {'Right','Left'};
% side_indx = 2; %left
        % Final Plotting
plotted_good = 0;
% log_file = fopen(strcat(xlfolder,'live_progress.txt'), 'w');
%% Iterate through each model selected
t_start = datetime('now'); 
% clear bone_indx side_folder_indx side_indx

nodes = vertices;
conlist = [];
nodes_original   = nodes;
conlist_original = conlist;

% Lists of different coordinate systems to choose from
list_talus = {'Talonavicular CS','Tibiotalar CS','Subtalar CS'};
list_calcaneus = {'Calcaneocuboid CS','Subtalar CS'};
list_metatarsal = {'Vertical Metatarsal CS','Radial Metatarsal CS'};
list_cuboid = {'Vertical Cuboid CS','Radial Cuboid CS'};
list_latcune = {'Vertical Lateral Cuneiform CS','Radial Lateral Cuneiform CS'};

% Ignore the bones that only have one coordinate system
need_cs_menu = ~(bone_indx == 3 || bone_indx == 5 || bone_indx == 6 || bone_indx == 13 || bone_indx == 14);

if (bone_indx == 3 || bone_indx == 5 || bone_indx == 6 || bone_indx == 13 || bone_indx == 14)
    bone_coord = 1;
    cs_string = "";
    
else
    if bone_indx == 1
        bone_coord = 2; %tibiotalar
        cs_string = string(list_talus(bone_coord));
    elseif bone_indx == 2
        bone_coord = [1,2];         
        cs_string = string(list_calcaneus(bone_coord));
    elseif bone_indx >= 8 && bone_indx <= 12
        bone_coord = [1,2];
        cs_string = string(list_metatarsal(bone_coord));
    elseif bone_indx == 4
        bone_coord = [1,2];
        cs_string = string(list_cuboid(bone_coord));
    elseif bone_indx == 7
        bone_coord = [1,2];
        cs_string = string(list_latcune(bone_coord));
    else
        bone_coord = 1;
        cs_string = "";
    end

    % if isempty(bone_coord) && need_cs_menu
    %     error('No coordinate system selected for %s. Operation cancelled.', FileName);
    % end
    % 
    % if need_cs_menu && apply_to_all
    %     cs_cache(int32(bone_indx)) = struct('bone_coord', bone_coord, 'cs_string', cs_string);
    % end
end

%% Loop for each desired Coordinate System
for n = 1:length(bone_coord)
    nodes = nodes_original;
    conlist = conlist_original;
    name = [];

    if side_indx == 1
        nodes = nodes.*[1,1,-1]; % Flip all rights to left
        conlist = [conlist(:,3) conlist(:,2) conlist(:,1)];
    end

    if bone_indx == 1
        list_joint = {'Center','Talonavicular Surface','Tibiotalar Surface', 'Subtalar Surface'};
    elseif bone_indx == 2
        list_joint = {'Center','Calcaneocuboid Surface', 'Subtalar Surface'};
    elseif bone_indx == 3
        list_joint = {'Center','Talonavicular Surface','Navicular-Cuneiform Surface'};
    elseif bone_indx == 4
        list_joint = {'Center','Calcaneocuboid Surface'};
    elseif bone_indx == 5 || bone_indx == 7
        list_joint = {'Center','Navicular-Cuneiform Surface', 'Cuneiform-Metatarsal Surface', 'Intercuneiform Surface'};
    elseif bone_indx == 6
        list_joint = {'Center','Navicular-Cuneiform Surface', 'Cuneiform-Metatarsal Surface', 'Medial Intercuneiform Surface', 'Lateral Intercuneiform Surface'};
    elseif bone_indx >= 8 && bone_indx <= 12
        list_joint = {'Center','Posterior Metatarsal Surface'};
    elseif bone_indx == 13
        list_joint = {'Center','Tibiotalar Surface'};
    elseif bone_indx == 14
        list_joint = {'Center','Talofibular Surface'};
    end


    joint_indx = 1; %centre 
    % Tibia/Fibula special handling
    if (bone_indx == 13 || bone_indx == 14)
        if numel(joint_indx) > 1
            bone_coord = 1:2;
        elseif joint_indx == 1
            bone_coord = 1;
        elseif joint_indx == 2
            bone_coord = 2;
        end
        cs_string = "";
    end

    %% ICP to Template
    % Align users model to the prealigned template model. This orients the
    % model in a fashion that the superior region is in the positive Z
    % direction, the anterior region is in the positive Y direction, and the
    % medial region is in the positive X direction.
    [nodes,cm_nodes] = center(nodes,1);
    better_start = 1; %standard
    better_start = 3; %better icp initial guess, pca alignment
    [aligned_nodes, RTs] = icp_template(bone_indx, nodes, bone_coord(n), better_start, name);

    %% Performs coordinate system calculation
    [Temp_Coordinates, Temp_Nodes] = CoordinateSystem(aligned_nodes, bone_indx, bone_coord(n), side_indx);

    %% Joint Origin
    if joint_indx > 1
        if isempty(conlist)
            Joint = "Center";
        else
            [Temp_Coordinates, Joint] = JointOrigin(Temp_Coordinates, Temp_Nodes, conlist, bone_indx, joint_indx, side_indx);
        end
    else
        Joint = "Center";
    end


    %% Temporarily Attach Coordinate System
    Temp_Nodes_Coords = [Temp_Nodes; Temp_Coordinates];

    %% Reorient and Translate to Original Input Origin and Orientation
    [nodes_final, coords_final, coords_final_unit, Temp_Coordinates_Unit] = reorient(Temp_Nodes_Coords, cm_nodes, side_indx, RTs);

    if bone_indx == 1 && bone_coord(n) == 3 % Additional alignment for talus subtalar ACS
        [aligned_nodes_TST, RTs_TST] = icp_template(bone_indx, nodes, 1, better_start, name);
        [Temp_Coordinates_TST, Temp_Nodes_TST] = CoordinateSystem(aligned_nodes_TST, bone_indx, 1, side_indx);

        if joint_indx > 1
            if isempty(conlist)
                Joint = "Center";
            else
                [Temp_Coordinates_TST, Joint] = JointOrigin(Temp_Coordinates_TST, Temp_Nodes_TST, conlist, bone_indx, joint_indx, side_indx);
            end
        else
            Joint = "Center";
        end

        Temp_Nodes_Coords_TST = [Temp_Nodes_TST; Temp_Coordinates_TST];

        [~, coords_final_TST, coords_final_unit_TST, Temp_Coordinates_Unit_TST] = reorient(Temp_Nodes_Coords_TST, cm_nodes, side_indx, RTs_TST);

        coords_final = [coords_final(1,:); ((coords_final_TST(2,:) + coords_final(2,:)).'/2)'
            coords_final(3,:); ((coords_final_TST(4,:) + coords_final(4,:)).'/2)'
            coords_final(5,:); ((coords_final_TST(6,:) + coords_final(6,:)).'/2)'];

        coords_final_unit = [coords_final_unit(1,:); ((coords_final_unit_TST(2,:) + coords_final_unit(2,:)).'/2)'
            coords_final_unit(3,:); ((coords_final_unit_TST(4,:) + coords_final_unit(4,:)).'/2)'
            coords_final_unit(5,:); ((coords_final_unit_TST(6,:) + coords_final_unit(6,:)).'/2)'];

        Temp_Coordinates_Unit = [Temp_Coordinates_Unit(1,:); ((Temp_Coordinates_Unit_TST(2,:) + Temp_Coordinates_Unit(2,:)).'/2)'
            Temp_Coordinates_Unit(3,:); ((Temp_Coordinates_Unit_TST(4,:) + Temp_Coordinates_Unit(4,:)).'/2)'
            Temp_Coordinates_Unit(5,:); ((Temp_Coordinates_Unit_TST(6,:) + Temp_Coordinates_Unit(6,:)).'/2)'];
    end

    origin = (coords_final_unit(1,:)); %origin
    AP = (coords_final_unit(2,:)); %AP
    SI = (coords_final_unit(4,:)); %SI
    ML = (coords_final_unit(6,:)); %ML

    if bone_indx == 1
        origin_point = origin;
        direction_point = ML;
        return
    end
    

    %% Clear Variables for New Loop
    vars = {'Temp_Nodes', 'Temp_Coordinates', 'Temp_Coordinates_Unit', 'Temp_Nodes_Coords', 'cm_nodes', 'RTs', 'coords_final','coords_final_unit','nodes','aligned_nodes','conlist'};
    clear(vars{:})
    t_end = datetime('now'); 
    true_elapsed = seconds(t_end - t_start);
    disp(strcat("elapsed time: ", num2str(true_elapsed), " seconds"))
    % fprintf(log_file, 'Case %s finished in %.4f seconds\n', name, true_elapsed);
end

end
