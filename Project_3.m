function varargout = Project_3(varargin)
% PROJECT1 M-file for project1.fig
%      PROJECT1, by itself, creates a new PROJECT1 or raises the existing
%      singleton*.
%
%      H = PROJECT1 returns the handle to a new PROJECT1 or the handle to
%      the existing singleton*.
%
%      PROJECT1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJECT1.M with the given input arguments.
%
%      PROJECT1('Property','Value',...) creates a new PROJECT1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before project1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to project1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help project1

% Last Modified by GUIDE v2.5 03-Dec-2018 17:32:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Project_3_OpeningFcn, ...
    'gui_OutputFcn',  @Project_3_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before project1 is made visible.
function Project_3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to project1 (see VARARGIN)

% Choose default command line output for project1
handles.output = hObject;
handles.class_robot = [];              % Our 3-axis manipulator
%created and use them as the max and min for the output. If we flip these,
%it effectively flips the direction of rotation for the servo
set(handles.x, 'String', 0);         % End-effector X position
set(handles.y, 'String', 0);         % End-effector Y position
set(handles.z, 'String', 0);         % End-effector Z position
set(handles.joint1, 'String', 0);    % Auxiliary variable for Joint 1 (used for text box input and output
set(handles.joint2, 'String', 0);    % Auxiliary variable for Joint 2 (used for text box input and output
set(handles.joint3, 'String', 0);    % Auxiliary variable for Joint 3 (used for text box input and output
handles.Algorithm=0;
handles.map=[];
handles.File_Name=[];
handles.SimulatedLightLocation=[];


% Update handles structure
guidata(hObject, handles);

% UIWAIT makes project1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Project_3_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in Create.
function Create_Callback(hObject, eventdata, handles)
% hObject    handle to Create (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%link creation
dh_Parameters=[0.053,0,0,-00.31416,0.025,0.11,0,-1.51844,0.0189,0.0885,-3.141593,0];
deg=pi/180;
Limits=[-74*deg,127.5*deg,-87*deg,114*deg,-120*deg,40*deg];
Base_Transform=[-0.132,0.135,0];
Tool_Transform=[0,0,0.0545];
Num_Links=3;
handles.class_robot=Path_Finder(dh_Parameters,Num_Links,Limits,Base_Transform,Tool_Transform);
axes(handles.axes1); %sets the active axis (1) for plotting
handles.class_robot.PlotRobot([0,45]); %Plots the robot with an azimuthal angle of 0 deg and and elevation of 45 deg
T = handles.class_robot.FkineRobot(); % Forward kinamtics using the initial values of the joint angles
% handles.x=T.t(1);% Set the x field to the value it gets from the default fkine
% handles.y=T.t(2);% Set the y field to the value it gets from the default fkine
% handles.z=T.t(3);% Set the z field to the value it gets from the default fkine
set(handles.x, 'String', T.t(1)); % Extracts the X component of the end-effector position from the transformation matrix
set(handles.y, 'String', T.t(2)); % Extracts the Y component of the end-effector position from the transformation matrix
set(handles.z, 'String', T.t(3)); % Extracts the Z component of the end-effector position from the transformation matrix
SoftORHard = questdlg('Would you like to use the hardware?', ...
    'Application Type', ...
    'Yes', 'No', 'No');
if strcmp(SoftORHard, 'Yes')
    handles.class_robot.Hardware_Used =1;
else
    handles.class_robot.Hardware_Used =0;
end


if (handles.class_robot.Hardware_Used ==0)
    LightSoftORHard = listdlg('ListString',{'Goal 1', 'Goal 2', 'Goal 3', 'Goal 4'},'PromptString','Where do you want the light source to be?','ListSize',[350,75]);
    if  LightSoftORHard == 1
        handles.SimulatedLightLocation =[0.080, 0.020];
    elseif LightSoftORHard == 2
        handles.SimulatedLightLocation =[0.070, 0.080];
    elseif LightSoftORHard == 3
        handles.SimulatedLightLocation =[0.010, 0.070];
    elseif LightSoftORHard == 4
        handles.SimulatedLightLocation =[0.020, 0.010];
    end
end
% Update handles structure
guidata(hObject, handles);
% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

handles.class_robot.Thetas(1) = get (hObject,'Value'); %returns position of slider in radians to theta 1
set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi); % displays value of theta 1 in Deg
T=handles.class_robot.MoveJoint(1,handles.class_robot.Thetas(1));%The zero and one are used to set the direction of the motor
set(handles.x, 'String', T.t(1)); % Extracts the X component of the end-effector position from the transformation matrix
set(handles.y, 'String', T.t(2)); % Extracts the Y component of the end-effector position from the transformation matrix
set(handles.z, 'String', T.t(3)); % Extracts the Z component of the end-effector position from the transformation matrix
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.class_robot.Thetas(2) = get(hObject, 'Value');
set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);
T=handles.class_robot.MoveJoint(2,handles.class_robot.Thetas(2));%The zero and one are used to set the direction of the motor
set(handles.x, 'String', T.t(1));
set(handles.y, 'String', T.t(2));
set(handles.z, 'String', T.t(3));
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.class_robot.Thetas(3) = get(hObject, 'Value');
set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);
T=handles.class_robot.MoveJoint(2,handles.class_robot.Thetas(3));%The zero and one are used to set the direction of the motor
set(handles.x, 'String', T.t(1));
set(handles.y, 'String', T.t(2));
set(handles.z, 'String', T.t(3));
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function joint1_Callback(hObject, eventdata, handles)
% hObject    handle to joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of joint1 as text
%        str2double(get(hObject,'String')) returns contents of joint1 as a double
handles.class_robot.Thetas(1) = str2num(get(handles.joint1, 'String'))*pi/180;%Grab the value that the user entered
handles.class_robot.Thetas(1)=handles.class_robot.CheckLimits(1,handles.class_robot.Thetas(1));%Check if the user typed theta is within the bounds for that joint. In this case, joint three
set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi);%Set the checked value to the GUI
T = handles.class_robot.MoveJoint(1,handles.class_robot.Thetas(1));%Recalculate the forward kinematics
set(handles.x, 'String', T.t(1));%Set the value for x on the gui from the forward kinematics
set(handles.y, 'String', T.t(2));%Set the value for y on the gui from the forward kinematics
set(handles.z, 'String', T.t(3));%Set the value for z on the gui from the forward kinematics
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function joint1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint2_Callback(hObject, eventdata, handles)
% hObject    handle to joint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint2 as text
%        str2double(get(hObject,'String')) returns contents of joint2 as a double
handles.class_robot.Thetas(2) = str2num(get(handles.joint2, 'String'))*pi/180;%Grab the value the user entered on the GUI
handles.class_robot.Thetas(2)=handles.class_robot.CheckLimits(2,handles.class_robot.Thetas(2));%Check if the user typed theta is within the bounds for that joint. In this case, joint three
set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
T = handles.class_robot.MoveJoint(2,handles.class_robot.Thetas(2));%Recalculate the forward kinematics
set(handles.x, 'String', T.t(1));%Set the value for x on the gui from the forward kinematics
set(handles.y, 'String', T.t(2));%Set the value for y on the gui from the forward kinematics
set(handles.z, 'String', T.t(3));%Set the value for z on the gui from the forward kinematics
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function joint2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint3_Callback(hObject, eventdata, handles)
% hObject    handle to joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint3 as text
%        str2double(get(hObject,'String')) returns contents of joint3 as a double
handles.class_robot.Thetas(3) = str2num(get(handles.joint3, 'String'))*pi/180;%Grab the user entered value from the GUI
handles.class_robot.Thetas(3)=handles.class_robot.CheckLimits(3,handles.class_robot.Thetas(3));%Check if the user typed theta is within the bounds for that joint. In this case, joint three
set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);%Set the display on the GUI to the new value if it was changed
T = handles.class_robot.MoveJoint(3,handles.class_robot.Thetas(3));%Recalculate the forward kinematics
set(handles.x, 'String', T.t(1));%Set the value for x on the gui from the forward kinematics
set(handles.y, 'String', T.t(2));%Set the value for y on the gui from the forward kinematics
set(handles.z, 'String', T.t(3));%Set the value for z on the gui from the forward kinematics
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function joint3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double
handles.class_robot.Tool_Position(1)=str2double(get(hObject,'String'));
handles.class_robot.MoveRobot(handles.class_robot.Tool_Position(1),handles.class_robot.Tool_Position(2),handles.class_robot.Tool_Position(3));
set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits

% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double
handles.class_robot.Tool_Position(2)=str2double(get(hObject,'String'));
handles.class_robot.MoveRobot(handles.class_robot.Tool_Position(1),handles.class_robot.Tool_Position(2),handles.class_robot.Tool_Position(3));
set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double
handles.class_robot.Tool_Position(3)=str2double(get(hObject,'String'));
handles.class_robot.MoveRobot(handles.class_robot.Tool_Position(1),handles.class_robot.Tool_Position(2),handles.class_robot.Tool_Position(3));
set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits

% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Workspace_Extents.
function Workspace_Extents_Callback(hObject, eventdata, handles)
% hObject    handle to Workspace_Extents (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Point=[0.0,0.0,0];
P=[];%This is needed because the array starts off as empty and we cannot append any values to a variable that isn't defined. So we define it here as a blank variable
for(i=handles.class_robot.Linked_Robot.qlim(1,1):15*pi/180:handles.class_robot.Linked_Robot.qlim(1,2))%Loop through all the first joint angles
    for(j=handles.class_robot.Linked_Robot.qlim(2,1):15*pi/180:handles.class_robot.Linked_Robot.qlim(2,2))%Loop through all the second joint angles
        %for(k=handles.class_robot.Linked_Robot.qlim(3,1):15*pi/180:handles.class_robot.Linked_Robot.qlim(3,2))%Loop through all the third joint angles
        T=handles.class_robot.Linked_Robot.fkine([i,j,0]);%Compute the forward kinematics on each angle combinations
        P=[P;T.transl];%Grab the translational part of the forward kinematics output adn append it to P in the next row. This will create a column vector of all the X,Y,and Z's
        %end
    end
end

axes(handles.axes1); %sets the active axis (1) for plotting
plot3(P(:,1),P(:,2),P(:,3),'rs');%Plot the points in the first view(Left hand view)
hold on;
plot3(Point(1),Point(2),Point(3),'bo')
handles.class_robot.PlotRobot([0,45]); %Plots the robot with an azimuthal angle of 0 deg and and elevation of 45 deg
zoom on;
% --- Executes on button press in Hardware_Menu


function Connect_Arduino_Callback(hObject, eventdata, handles)
% hObject    handle to Hardware_Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Hardware = char (inputdlg ({'What COM Port?' , 'What type of Board?' }, 'Hardware Select', [1,35],{'COM6','Uno'}));
%BoardType= char (inputdlg ('What type of Board' , 'Board Select', 1, {'uno'}));
COMPort=strtrim(Hardware(1,:));%Trim off unessecary spaces that may have gotten in there
BoardType=strtrim(Hardware(2,:));%Trim off unessecary spaces that may have gotten in there
try%Catch any errors that may pop up from the attempts to access the arduino
    handles.class_robot.ConnectHardware(COMPort,BoardType);%Call the function to connect the hardware from the robot class
catch%If an error was thrown, we catch it here and display to the user that that com port was invalid or unavailable
    msgbox("There is no Arduino on that COM port","ERROR");%Display the error to the user
    uiwait();%Make the GUI wait for the user to deal with the box. This forces them to read the error
end

guidata(hObject, handles);%Update the GUI data

function Add_Servo_Callback(hObject, eventdata, handles)
% hObject    handle to Add_Servo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Num_Add_Servos=str2double(char(inputdlg('Enter the number of servos you would like to add','Add Servos')));

for(i=1:1:Num_Add_Servos)
    Message1=['Please enter the pin number that servo ',num2str(i),' is on'];
    Message2=['Please enter the minimum pulse duration for servo ',num2str(i)];
    Message3=['Please enter the maximum pulse duration for servo ',num2str(i)];
    Message4=['Is this servo replacing another?'];
    Message5=['What position is this servo going in?'];
    
    Servo_Config=inputdlg({Message1,Message2,Message3,Message4,Message5},['Servo ',num2str(i)],[1,35],{'D3','575e-6','2460e-6','No','Last'});
    Pin_Number=char(strtrim(Servo_Config(1)));
    Min=str2double(char(strtrim(Servo_Config(2))));
    Max=str2double(char(strtrim(Servo_Config(3))));
    Replace=strtrim(Servo_Config(4));
    Position=strtrim(Servo_Config(5));
    handles.class_robot.AddServo(Pin_Number,Min,Max,Position,Replace)
    handles.class_robot.MoveJoint(i,0);
    
end
guidata(hObject,handles);


% --------------------------------------------------------------------
function Detach_Servo_Callback(hObject, eventdata, handles)
% hObject    handle to Detach_Servo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Servo_to_Delete=str2double(char(inputdlg('Enter the servo number that you would like to remove')));
handles.class_robot.DetachServo(Servo_to_Delete);
guidata(hObject,handles);


% --------------------------------------------------------------------
function Detach_Hardware_Callback(hObject, eventdata, handles)
% hObject    handle to Detach_Hardware (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Confirmed=listdlg('ListString',{'Yes','No'},'PromptString','Are you sure you wish to remove ALL hardware from the robot?','ListSize',[350,75]);
if(Confirmed ==1)
    handles.class_robot.DetachHardware();
else
    msgbox('Hardware will not be removed from robot!');
end
guidata(hObject,handles);


% --- Executes on button press in Load_Image.
function Load_Image_Callback(hObject, eventdata, handles)
% hObject    handle to Load_Image (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes2) %sets the active axies (2) for displaying the image
handles.File_Name=handles.class_robot.LoadImage();
set(handles.File_Destination,'string',handles.File_Name);
guidata(hObject,handles);



function File_Destination_Callback(hObject, eventdata, handles)
% hObject    handle to File_Destination (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of File_Destination as text
%        str2double(get(hObject,'String')) returns contents of File_Destination as a double


% --- Executes during object creation, after setting all properties.
function File_Destination_CreateFcn(hObject, eventdata, handles)
% hObject    handle to File_Destination (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Paint.
function Find_Light_Callback(hObject, eventdata, handles)
% hObject    handle to Paint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);% Set the current axis to axes1, this is the robot graph
Original_Start=[0.045,0.045];% holds the location of the start location
Current_Location=[0.045,0.045];% holds the current location that we are at
Z=0.0046;% Variable to hold the height that the end effector is set from the surface
handles.class_robot.MoveRobot(-Original_Start(1),Original_Start(2),Z);% Move the robot to all the points on the path
T = handles.class_robot.MoveJoint(2,handles.class_robot.Thetas(2));%Recalculate the forward kinematics
set(handles.x, 'String', T.t(1));%Set the value for x on the gui from the forward kinematics
set(handles.y, 'String', T.t(2));%Set the value for y on the gui from the forward kinematics
set(handles.z, 'String', T.t(3));%Set the value for z on the gui from the forward kinematics
set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits

for(i=1:4)
    switch(i)
        case 1
            Goal=[0.080,0.020];
            
        case 2
            Goal=[0.070,0.080];
            
        case 3
            Goal=[0.010,0.070];
            
        case 4
            Goal=[0.020,0.010];
            
        otherwise
            % Do Nothing
    end
    
    [Points,s]=handles.class_robot.PlanPath(Current_Location*1000,Goal*1000,handles.map,handles.Algorithm);% Get the points needed to move from the current location to the goal
    Points=Points/1000;
    Points(:,1)=-1*Points(:,1);% Convert the x values to be negative because thats how our board is set up
    axes(handles.axes2);% Change the axis to axes 2, which os the map
    for(i=1:numel(s)/2)
        hold on
        plot(s(i,1),s(i,2),'ws');% Plot
    end
    axes(handles.axes1);% Set the axis back to 1, which is the robot plot
    for(n=1:(numel(Points)/2))% Loop through all the points
        handles.class_robot.MoveRobot(Points(n,1),Points(n,2),Z);% Move the robot to all the points on the path
        T = handles.class_robot.MoveJoint(2,handles.class_robot.Thetas(2));%Recalculate the forward kinematics
        set(handles.x, 'String', T.t(1));%Set the value for x on the gui from the forward kinematics
        set(handles.y, 'String', T.t(2));%Set the value for y on the gui from the forward kinematics
        set(handles.z, 'String', T.t(3));%Set the value for z on the gui from the forward kinematics
        set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
        set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
        set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
        
        axes(handles.axes2);
        hold on;
        plot(-1*Points(n,1)*1000,Points(n,2)*1000,'rs');
        axes(handles.axes1);
    end
    
    if (handles.class_robot.Hardware_Used == 1)
        handles.class_robot.CheckLightSensor();
        if(handles.class_robot.Found_Light_State==1)% If light was detected, then we will break out of the loop and go to the finish
            Current_Location=Goal;% Update the current location to the where the current goal is because we have finished the move and are at the goal location
            break;
        end
    else
        if(Goal == handles.SimulatedLightLocation)
            Current_Location=Goal;% Update the current location to the where the current goal is because we have finished the move and are at the goal location
            handles.class_robot.Display_Emotion();
            break;
        end
    end
    Current_Location=Goal;% Update the current location to the where the current goal is because we have finished the move and are at the goal location
    
end
% Once we have finished searching through the boxes or have found a light,
% then we will set the goal as the original starting location and move from
% the current location to it
Goal=Original_Start;% Set the goal tpo the original starting location
[Points,s]=handles.class_robot.PlanPath(Current_Location*1000,Goal*1000,handles.map,handles.Algorithm);% Get the points needed to move from the current location to the goal
Points=Points/1000;
Points(:,1)=-1*Points(:,1);% Convert the x values to be negative because thats how our board is set up
axes(handles.axes2);% Change the axis to axes 2, which os the map
for(i=1:numel(s)/2)
    hold on
    plot(s(i,1),s(i,2),'ws');% Plot
end
axes(handles.axes1);% Set the axis back to 1, which is the robot plot
for(n=1:size(Points))% Loop through the path points and move the robot through each
    handles.class_robot.MoveRobot(Points(n,1),Points(n,2),Z);% Move the robot to all the points on the path
    T = handles.class_robot.MoveJoint(2,handles.class_robot.Thetas(2));%Recalculate the forward kinematics
    set(handles.x, 'String', T.t(1));%Set the value for x on the gui from the forward kinematics
    set(handles.y, 'String', T.t(2));%Set the value for y on the gui from the forward kinematics
    set(handles.z, 'String', T.t(3));%Set the value for z on the gui from the forward kinematics
    set(handles.joint1, 'String', handles.class_robot.Thetas(1)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
    set(handles.joint2, 'String', handles.class_robot.Thetas(2)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
    set(handles.joint3, 'String', handles.class_robot.Thetas(3)*180/pi);%Set the display to the checked theta value of it was changed or display the same number if it was within limits
    axes(handles.axes2);
    hold on;
    plot(-1*Points(n,1)*1000,Points(n,2)*1000,'bo');
    axes(handles.axes1);
end
msgbox('Found the light, and have returned home');


% --- Executes on button press in ConvertImage.
function ConvertImage_Callback(hObject, eventdata, handles)
axes(handles.axes2);
handles.map=handles.class_robot.ConvertImagetoExclusion(imread(handles.File_Name));
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function SelectAlgorithm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SelectFeature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in SelectFeature.
function SelectFeature_Callback(hObject, eventdata, handles)
% hObject    handle to SelectFeature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.contents = cellstr(get(hObject,'String'));
Choice = handles.contents{get(hObject,'Value')};

if strcmp(Choice,'D-Star')
    handles.Algorithm = 0;% If the user selected the A-Star algorithm, use the A-star algorithm
else
    handles.Algorithm = 1;% otherwise, we should select the Probability Road Map algorithm
end
guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns SelectFeature contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SelectFeature
