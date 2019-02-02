classdef Path_Finder < Robot_Class
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Flag;% Flag to start/stop drawing process
        Found_Light_State;
    end
    
    methods
        function obj = Path_Finder(dh,Num_Links,Limits,Base_Transform,Tool_Transform)
            obj@Robot_Class(dh,Num_Links,Limits,Base_Transform,Tool_Transform);% Call the constructor from the parent class Robot_Class
            obj.Flag = 0;
            obj.Found_Light_State = 0;
        end
        
        function File_Name=LoadImage(obj)
            %Delete any previously created file containing the positions to
            %draw by the Artist Robot.
            if exist('positions2go.xlsx', 'file')
                delete('positions2go.xlsx');
            end
            %The user select the JPG image from the hard disk, its
            %destination is concatinated with the file name.
            [fn, pn] = uigetfile('*.jpg','Select an image');
            File_Name_Destination = strcat(pn,fn);
            
            %Set the 3D matrix I to contain the image (in the form of pixel by
            I = imread(File_Name_Destination); %Read the image
            I=imbinarize(I);% Convert the image to a binary image
            imshow(I); %Show the image in the assigned GUI axes
            
            File_Name=File_Name_Destination;
        end
        
        function map=ConvertImagetoExclusion(obj,Binary_Image)
            map=imbinarize(Binary_Image);% Convert the image to only 0's and 1's
            map=Convert_to_Exclusion_Map(map,2);% Convert the binary image to an exclusion map with a gap on each side of 2
            imshow(map); % Show the exclusion map on the screen
        end
        
        function CheckLightSensor(obj)
            threshold=1;% variable to hold the threshold for the robot. This is the sensitivity for the robot; a lower value will trigger more often and might lead to false positives. A higher value will be
            %less prone to false positives, but it may miss an actual light source
            for(i=obj.Linked_Robot.qlim(3,1):15*pi/180:obj.Linked_Robot.qlim(3,2))% Loop through the joint angles in 15 degree increments
                obj.MoveJoint(3,i);% Move the 3rd joint to the specified angle
                if(obj.GetLightSensorValue() > threshold)% If the light sensor value is above the threshold, then we will say we have found a light
                    obj.Found_Light_State = 1;
                    obj.Display_Emotion();
                    break;% If we have found the light, we should stop the scan
                else
                    obj.Found_Light_State = 0;
                    obj.Display_Emotion();
                end
            end
        end
        
        
        function Display_Emotion(obj) %Function to turn on the light led and sound buzzer
            if (obj.Hardware_Used == 1)
                if (obj.Found_Light_State) %Check if Buzzer and Light state
                    writeDigitalPin(obj.Controller,obj.Buzzer_Pin,1);
                    writeDigitalPin(obj.Controller,obj.LED_Pin,1);
                elseif (~obj.Found_Light_State)
                    writeDigitalPin(obj.Controller,obj.Buzzer_Pin,0);
                    writeDigitalPin(obj.Controller,obj.LED_Pin,0);
                end
            else
                msgbox("The sound buzzer and LED are ON");
                uiwait();
            end
        end      
    end
end

