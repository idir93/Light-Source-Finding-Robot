function Map = Convert_to_Exclusion_Map(Binary_Image,Exclusion_Width)
%CONVERT_TO_EXCLUSION_MAP: Function to convert a binary image to an
%exclusion map
% Converts a binary image to a exclusion map with a border of ones around
% it to avoid the robot from potentially colliding with the wall
[Height, Width]=size(Binary_Image);% get the width and height of the binary image
Locations=[];% Set the locations that contain a 1 to the null array
for(i=1:1:Height)% Loop through the rows
    for(j=1:1:Width)% loop through the columns
        if(Binary_Image(i,j) == 1)% Check if the value at that location is a one
            Locations=[Locations;i,j];% Store the location in an array
        end
    end
end

[Rows, ~]=size(Locations);
Places_to_Turn=zeros((2*Exclusion_Width+1)^2-1,2);
z=1;% set z = 1, this variable helps to figure out where to store each location into an array
for(i=1:1:Rows)% scan through all the points that were ones to find the points that should be added to the exclusion map
    for(t=1:1:Exclusion_Width)% Loop through each layer surrounding the current value
        LC=Locations(i,:)-[t,t];% Get the location of the upper left corner
        RC=Locations(i,:)+[-t,t];% get the location of the upper right corner
        
        if(LC(1) > 0 && LC(1) <= Height && LC(2) > 0 && LC(2) <= Width)%  If the upper left corner is a valid location, then we will store them, otherwise, don't.
        Places_to_Turn(z,:)=LC;% We know that the left corner should be a one, store its value
        z=z+1;% increment z by 1 to store the next location to turn
        end
        
         if(RC(1) > 0 && RC(1) <= Height && RC(2) > 0 && RC(2) <= Width)% If the upper right corner is a valid location, then we will store them, otherwise, don't.
        Places_to_Turn(z,:)=RC;% We know that the left corner should be a one, store its value
        end
        
        
        for(m=1:4)% loop through all the sides of the box
            switch(m)% switch depending on which side we are on
                case 1
                    for(j=1:1:2*t)
                        z=z+1;% Increment z by 1
                        Location=LC+[j,0];
                        if(Location(1) > 0 && Location(1) <= Height && Location(2) > 0 && Location(2) <= Width)% Check if the location is valid
                            Places_to_Turn(z,:)=Location;% Find the location and store it
                        end
                    end
                    
                case 2
                    for(j=1:2*t-1)
                        z=z+1;% Increment z by 1
                        Location = LC+[2*t,j];
                        if(Location(1) > 0 && Location(1) <= Height && Location(2) > 0 && Location(2) <= Width)% Check if the location is valid
                            Places_to_Turn(z,:)=Location;% Find the location and store it
                        end
                    end
                    
                case 3
                    for(j=1:1:2*t)
                        z=z+1;% Increment z by 1
                        Location=RC+[j,0];
                        if(Location(1) > 0 && Location(1) <= Height && Location(2) > 0 && Location(2) <= Width)% Check if the location is valid
                            Places_to_Turn(z,:)=Location;% Find the location and store it
                        end
                    end
                    
                case 4
                    for(j=1:1:2*t-1)
                        z=z+1;% Increment z by 1
                        Location=RC+[0,-j];
                        if(Location(1) > 0 && Location(1) <= Height && Location(2) > 0 && Location(2) <= Width)% Check if the location is valid
                            Places_to_Turn(z,:)=Location;% Find the location and store it
                        end
                    end
                    
                otherwise
                    %Do nothing and skip to next iteration
            end
        end
        z=z+1;% Increment z by 1 because we are going to a different layer
    end
end% end scanning for locations to turn


Places_to_Turn=Places_to_Turn(any(Places_to_Turn,2),:);% remove any row that had a zero in it since it is invalid anyways
[Num_Rows,~]=size(Places_to_Turn);% get the size of the locations to turn array

for(i=1:1:Num_Rows)
    Binary_Image(Places_to_Turn(i,1),Places_to_Turn(i,2))=1;% Change each location to a 1
end

Map=Binary_Image;% set the output to the changed binary image
end

