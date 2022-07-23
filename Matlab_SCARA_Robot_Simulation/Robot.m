function varargout = Robot(varargin)
% ROBOT MATLAB code for Robot.fig
%      ROBOT, by itself, creates a new ROBOT or raises the existing
%      singleton*.
%
%      H = ROBOT returns the handle to a new ROBOT or the handle to
%      the existing singleton*.
%
%      ROBOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOT.M with the given input arguments.
%
%      ROBOT('Property','Value',...) creates a new ROBOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Robot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Robot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Robot

% Last Modified by GUIDE v2.5 29-Dec-2021 16:20:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Robot_OpeningFcn, ...
                   'gui_OutputFcn',  @Robot_OutputFcn, ...
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


% --- Executes just before Robot is made visible.
function Robot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Robot (see VARARGIN)

% Choose default command line output for Robot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global theta1 theta2 d3 theta4
theta1 = 0.0;
theta2 = pi/2;
d3 = 0.0;
theta4 = 0.0;
set(handles.slider_theta2,'value',theta2/pi*180);
% if (isempty(theta1))
%     theta1 = 0.0;
%     theta2 = 0.0;
%     d3 = 0.0;
%     theta4 = 0.0;
% end
global a1 a2 d1 d4
a1     = 0.45;
a2     = 0.4;
d1     = 0.4605;
d4     = -0.085;

%forward_kinematics(handles);

%inverse_kinematics(handles);
%forward_kinematics(handles);
global enable_workspace_visible
enable_workspace_visible = false;

set(handles.edit_v_max,'String',0.1*1000);
set(handles.edit_a_max,'String',0.1*1000);

set(handles.rdiobtn_LSPB,'Value',1);
set(handles.rdiobtn_S_curve,'Value',0);
set(handles.rdiobtn_path_linear,'Value',1);
set(handles.rdiobtn_path_circular,'Value',0);
global mode_trajectory mode_path
mode_trajectory = 0;
mode_path = 0;
%--------------------
global disable_kinematics_singularity
disable_kinematics_singularity = false;
set(handles.cbox_disable_kinematics_singularity,'Value',0);
global enable_noncentralize_control
enable_noncentralize_control = false;
set(handles.cbox_enable_noncentralize_control,'Value',0);
%--------------------
global mode_RobotView
mode_RobotView = 1;
set(handles.rdiobtn_RobotCubesView,'Value',1);
set(handles.rdiobtn_RobotLinesView,'Value',0);
global value_3dRobotView_opacity
value_3dRobotView_opacity = 0.5;
set(handles.slider_3dRobotView_opacity,'Value',value_3dRobotView_opacity)
set(handles.edit_3dRobotView_opacity,'String',value_3dRobotView_opacity)
%--------------------
global graph_position;
graph_position = [89.14285714285714 16.058823529411764 151.7142857142857 33.23529411764706];
set(handles.uipanel_toolspace_graph,'visible','on','position',graph_position);
set(handles.uipanel_jointspace_graph,'visible','off','position',graph_position);
%--------------------
global graph_indication_position;
graph_indication_position = [209.0 49.05882352941176 31.285714285714278 4.058823529411761];
set(handles.uipanel_graph_indication,'visible','off','position',graph_indication_position);

hold(handles.axes_q,'on');
hold(handles.axes_qx,'on');
hold(handles.axes_qy,'on');
hold(handles.axes_qz,'on');
hold(handles.axes_v,'on');
hold(handles.axes_vx,'on');
hold(handles.axes_vy,'on');
hold(handles.axes_vz,'on');
hold(handles.axes_a,'on');
hold(handles.axes_ax,'on');
hold(handles.axes_ay,'on');
hold(handles.axes_az,'on');
hold(handles.axes_theta1,'on');
hold(handles.axes_theta2,'on');
hold(handles.axes_d3,'on');
hold(handles.axes_theta4,'on');
hold(handles.axes_theta1_dot,'on');
hold(handles.axes_theta2_dot,'on');
hold(handles.axes_d3_dot,'on');
hold(handles.axes_theta4_dot,'on');
hold(handles.axes_theta1_2dot,'on');
hold(handles.axes_theta2_2dot,'on');
hold(handles.axes_d3_2dot,'on');
hold(handles.axes_theta4_2dot,'on');

title(handles.axes_q,'q(mm)');
title(handles.axes_qx,'qx(mm)');
title(handles.axes_qy,'qy(mm)');
title(handles.axes_qz,'qz(mm)');
title(handles.axes_v,'v(mm/s)');
title(handles.axes_vx,'vx(mm/s)');
title(handles.axes_vy,'vy(mm/s)');
title(handles.axes_vz,'vz(mm/s)');
title(handles.axes_a,'a(mm/s2)');
title(handles.axes_ax,'ax(mm/s2)');
title(handles.axes_ay,'ay(mm/s2)');
title(handles.axes_az,'az(mm/s2)');
title(handles.axes_theta1,'theta1(rad)');
title(handles.axes_theta2,'theta2(rad)');
title(handles.axes_d3,'d3(mm)');
title(handles.axes_theta4,'theta4(rad)');
title(handles.axes_theta1_dot,'theta1 dot(rad/s)');
title(handles.axes_theta2_dot,'theta2 dot(rad/s)');
title(handles.axes_d3_dot,'d3 dot(mm/s)');
title(handles.axes_theta4_dot,'theta4 dot(rad/s)');
title(handles.axes_theta1_2dot,'theta1 2dot(rad/s2)');
title(handles.axes_theta2_2dot,'theta2 2dot(rad/s2)');
title(handles.axes_d3_2dot,'d3 2dot(mm/s2)');
title(handles.axes_theta4_2dot,'theta4 2dot(rad/s2)');

grid(handles.axes_q,'on');
grid(handles.axes_qx,'on');
grid(handles.axes_qy,'on');
grid(handles.axes_qz,'on');
grid(handles.axes_v,'on');
grid(handles.axes_vx,'on');
grid(handles.axes_vy,'on');
grid(handles.axes_vz,'on');
grid(handles.axes_a,'on');
grid(handles.axes_ax,'on');
grid(handles.axes_ay,'on');
grid(handles.axes_az,'on');
grid(handles.axes_theta1,'on');
grid(handles.axes_theta2,'on');
grid(handles.axes_d3,'on');
grid(handles.axes_theta4,'on');
grid(handles.axes_theta1_dot,'on');
grid(handles.axes_theta2_dot,'on');
grid(handles.axes_d3_dot,'on');
grid(handles.axes_theta4_dot,'on');
grid(handles.axes_theta1_2dot,'on');
grid(handles.axes_theta2_2dot,'on');
grid(handles.axes_d3_2dot,'on');
grid(handles.axes_theta4_2dot,'on');


title(handles.axes_robot,'Robot View');
hold(handles.axes_robot,'on')
grid(handles.axes_robot,'on')
xlabel(handles.axes_robot,'x(m)');
ylabel(handles.axes_robot,'y(m)');
zlabel(handles.axes_robot,'z(m)');
% xlim(handles.axes_robot,[-1 1]);
% ylim(handles.axes_robot,[-1 1]);
% zlim(handles.axes_robot,[0 1]);
axis(handles.axes_robot,[-1 1 -1 1 0 1.15])
rotate3d(handles.axes_robot,'on')
axis(handles.axes_robot,'equal')
% axes(handles.axes_robot);
update_end_effector(theta1, theta2, d3, theta4, handles);
view(handles.axes_robot,[45 45])
%--------------------
global DynamicModel
DynamicModel = 'Dynamic';
load_system(DynamicModel);
open_system(DynamicModel); 
% UIWAIT makes Robot wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Robot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_theta1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%a=1;
forward_kinematics(handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_theta1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_theta1 as text
%        str2double(get(hObject,'String')) returns contents of edit_theta1 as a double
theta1_new = str2double(get(handles.edit_theta1,'string'));
set(handles.slider_theta1,'value',theta1_new);
forward_kinematics(handles);

% --- Executes during object creation, after setting all properties.
function edit_theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_theta2_Callback(hObject, eventdata, handles)
% hObject    handle to slider_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
forward_kinematics(handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_theta2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_theta2 as text
%        str2double(get(hObject,'String')) returns contents of edit_theta2 as a double
theta2_new = str2double(get(handles.edit_theta2,'string'));
set(handles.slider_theta2,'value',theta2_new);
forward_kinematics(handles);

% --- Executes during object creation, after setting all properties.
function edit_theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_d3_Callback(hObject, eventdata, handles)
% hObject    handle to slider_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
forward_kinematics(handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_d3_Callback(hObject, eventdata, handles)
% hObject    handle to edit_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_d3 as text
%        str2double(get(hObject,'String')) returns contents of edit_d3 as a double
d3_new = str2double(get(handles.edit_d3,'string'));
set(handles.slider_d3,'value',d3_new);
forward_kinematics(handles);

% --- Executes during object creation, after setting all properties.
function edit_d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_theta4_Callback(hObject, eventdata, handles)
% hObject    handle to slider_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
forward_kinematics(handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_theta4_Callback(hObject, eventdata, handles)
% hObject    handle to edit_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_theta4 as text
%        str2double(get(hObject,'String')) returns contents of edit_theta4 as a double
theta4_new = str2double(get(handles.edit_theta4,'string'));
set(handles.slider_theta4,'value',theta4_new);
forward_kinematics(handles);

% --- Executes during object creation, after setting all properties.
function edit_theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_x_Callback(hObject, eventdata, handles)
% hObject    handle to edit_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_x as text
%        str2double(get(hObject,'String')) returns contents of edit_x as a double


% --- Executes during object creation, after setting all properties.
function edit_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_y_Callback(hObject, eventdata, handles)
% hObject    handle to edit_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_y as text
%        str2double(get(hObject,'String')) returns contents of edit_y as a double


% --- Executes during object creation, after setting all properties.
function edit_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_z_Callback(hObject, eventdata, handles)
% hObject    handle to edit_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_z as text
%        str2double(get(hObject,'String')) returns contents of edit_z as a double


% --- Executes during object creation, after setting all properties.
function edit_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_roll_Callback(hObject, eventdata, handles)
% hObject    handle to edit_roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_roll as text
%        str2double(get(hObject,'String')) returns contents of edit_roll as a double


% --- Executes during object creation, after setting all properties.
function edit_roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pitch_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pitch as text
%        str2double(get(hObject,'String')) returns contents of edit_pitch as a double


% --- Executes during object creation, after setting all properties.
function edit_pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yaw_Callback(hObject, eventdata, handles)
% hObject    handle to edit_yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_yaw as text
%        str2double(get(hObject,'String')) returns contents of edit_yaw as a double


% --- Executes during object creation, after setting all properties.
function edit_yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1 theta2 d3 theta4
global disable_kinematics_singularity
if (isempty(theta1))
    theta1 = 0.0;
    theta2 = 0.0;
    d3 = 0.0;
    theta4 = 0.0;
end
px = str2double(get(handles.edit_x,'string'));
py = str2double(get(handles.edit_y,'string'));
pz = str2double(get(handles.edit_z,'string'));
% roll = str2double(get(handles.edit_roll,'string'))*pi/180;
% pitch = str2double(get(handles.edit_pitch,'string'))*pi/180;
yaw = str2double(get(handles.edit_yaw,'string'))*pi/180;
inverse_kinematics(px,py,pz,yaw,disable_kinematics_singularity,true);
update_end_effector(theta1, theta2, d3, theta4, handles);



% --- Executes on button press in cbox_workspace.
function cbox_workspace_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_workspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1 theta2 d3 theta4
if (isempty(theta1))
    theta1 = 0.0;
    theta2 = 0.0;
    d3 = 0.0;
    theta4 = 0.0;
end
global enable_workspace_visible
if (get(handles.cbox_workspace,'value')<0.5)
    enable_workspace_visible = false;
    cla
    update_end_effector(theta1, theta2, d3, theta4, handles);
else
    enable_workspace_visible = true;
    plot_workspace(handles);
end

% Hint: get(hObject,'Value') returns toggle state of cbox_workspace



% --- Executes on button press in btn_planning.
function btn_planning_Callback(hObject, eventdata, handles)
% hObject    handle to btn_planning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mode_trajectory mode_path
global disable_kinematics_singularity
global enable_noncentralize_control
xA = str2double(get(handles.edit_x,'string'));
yA = str2double(get(handles.edit_y,'string'));
zA = str2double(get(handles.edit_z,'string'));
yawA = str2double(get(handles.edit_yaw,'string'));
xB = str2double(get(handles.edit_x_next,'string'));
yB = str2double(get(handles.edit_y_next,'string'));
zB = str2double(get(handles.edit_z_next,'string'));
yawB = str2double(get(handles.edit_yaw_next,'string'))*pi/180;
PathPlanning(xA,yA,zA,yawA,xB,yB,zB,yawB,mode_trajectory,mode_path,disable_kinematics_singularity,enable_noncentralize_control,handles);


function edit_x_next_Callback(hObject, eventdata, handles)
% hObject    handle to edit_x_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_x_next as text
%        str2double(get(hObject,'String')) returns contents of edit_x_next as a double


% --- Executes during object creation, after setting all properties.
function edit_x_next_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_x_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_y_next_Callback(hObject, eventdata, handles)
% hObject    handle to edit_y_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_y_next as text
%        str2double(get(hObject,'String')) returns contents of edit_y_next as a double


% --- Executes during object creation, after setting all properties.
function edit_y_next_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_y_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_z_next_Callback(hObject, eventdata, handles)
% hObject    handle to edit_z_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_z_next as text
%        str2double(get(hObject,'String')) returns contents of edit_z_next as a double


% --- Executes during object creation, after setting all properties.
function edit_z_next_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_z_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in rdiobtn_path_circular.
function rdiobtn_path_circular_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_path_circular (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mode_path
mode_path = 1;
set(handles.rdiobtn_path_linear,'Value',0);
% Hint: get(hObject,'Value') returns toggle state of rdiobtn_path_circular


% --- Executes on button press in rdiobtn_path_linear.
function rdiobtn_path_linear_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_path_linear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mode_path
mode_path = 0;
set(handles.rdiobtn_path_circular,'Value',0);
% Hint: get(hObject,'Value') returns toggle state of rdiobtn_path_linear


% --- Executes on button press in btn_graph_jointspace.
function btn_graph_jointspace_Callback(hObject, eventdata, handles)
% hObject    handle to btn_graph_jointspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global graph_position;
set(handles.uipanel_toolspace_graph,'visible','off','position',graph_position);
set(handles.uipanel_jointspace_graph,'visible','on','position',graph_position);


% --- Executes on button press in btn_graph_toolspace.
function btn_graph_toolspace_Callback(hObject, eventdata, handles)
% hObject    handle to btn_graph_toolspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global graph_position;
set(handles.uipanel_toolspace_graph,'visible','on','position',graph_position);
set(handles.uipanel_jointspace_graph,'visible','off','position',graph_position);


% --- Executes on button press in rdiobtn_S_curve.
function rdiobtn_S_curve_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_S_curve (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mode_trajectory
mode_trajectory = 1;
set(handles.rdiobtn_LSPB,'Value',0);
% Hint: get(hObject,'Value') returns toggle state of rdiobtn_S_curve


% --- Executes on button press in rdiobtn_LSPB.
function rdiobtn_LSPB_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_LSPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mode_trajectory
mode_trajectory = 0;
set(handles.rdiobtn_S_curve,'Value',0);
% Hint: get(hObject,'Value') returns toggle state of rdiobtn_LSPB



function edit_roll_next_Callback(hObject, eventdata, handles)
% hObject    handle to edit_roll_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_roll_next as text
%        str2double(get(hObject,'String')) returns contents of edit_roll_next as a double


% --- Executes during object creation, after setting all properties.
function edit_roll_next_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_roll_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pitch_next_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pitch_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pitch_next as text
%        str2double(get(hObject,'String')) returns contents of edit_pitch_next as a double


% --- Executes during object creation, after setting all properties.
function edit_pitch_next_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pitch_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yaw_next_Callback(hObject, eventdata, handles)
% hObject    handle to edit_yaw_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_yaw_next as text
%        str2double(get(hObject,'String')) returns contents of edit_yaw_next as a double


% --- Executes during object creation, after setting all properties.
function edit_yaw_next_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_yaw_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cbox_view_path.
function cbox_view_path_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_view_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbox_view_path


% --- Executes on button press in btn_reset.
function btn_reset_Callback(hObject, eventdata, handles)
% hObject    handle to btn_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in rdiobtn_full_path.
function rdiobtn_full_path_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_full_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rdiobtn_full_path


% --- Executes on button press in rdiobtn_nearest_path.
function rdiobtn_nearest_path_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_nearest_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rdiobtn_nearest_path



function edit_q_max_Callback(hObject, eventdata, handles)
% hObject    handle to edit_q_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_q_max as text
%        str2double(get(hObject,'String')) returns contents of edit_q_max as a double


% --- Executes during object creation, after setting all properties.
function edit_q_max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_q_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_v_max_Callback(hObject, eventdata, handles)
% hObject    handle to edit_v_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_v_max as text
%        str2double(get(hObject,'String')) returns contents of edit_v_max as a double


% --- Executes during object creation, after setting all properties.
function edit_v_max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_v_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_a_max_Callback(hObject, eventdata, handles)
% hObject    handle to edit_a_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_a_max as text
%        str2double(get(hObject,'String')) returns contents of edit_a_max as a double


% --- Executes during object creation, after setting all properties.
function edit_a_max_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_a_max (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cbox_disable_kinematics_singularity.
function cbox_disable_kinematics_singularity_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_disable_kinematics_singularity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global disable_kinematics_singularity
if (get(handles.cbox_disable_kinematics_singularity,'Value') > 0.5)
    disable_kinematics_singularity = true;
else
    disable_kinematics_singularity = false;
end
% Hint: get(hObject,'Value') returns toggle state of cbox_disable_kinematics_singularity


% --- Executes on slider movement.
function slider_3dRobotView_opacity_Callback(hObject, eventdata, handles)
% hObject    handle to slider_3dRobotView_opacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global value_3dRobotView_opacity
value_3dRobotView_opacity = get(handles.slider_3dRobotView_opacity,'Value');
set(handles.edit_3dRobotView_opacity,'String',value_3dRobotView_opacity)
global theta1 theta2 d3 theta4
update_end_effector(theta1, theta2, d3, theta4, handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_3dRobotView_opacity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_3dRobotView_opacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_3dRobotView_opacity_Callback(hObject, eventdata, handles)
% hObject    handle to edit_3dRobotView_opacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global value_3dRobotView_opacity
value_3dRobotView_opacity = str2double(get(handles.edit_3dRobotView_opacity,'String'));
set(handles.slider_3dRobotView_opacity,'Value',value_3dRobotView_opacity)
global theta1 theta2 d3 theta4
update_end_effector(theta1, theta2, d3, theta4, handles);
% Hints: get(hObject,'String') returns contents of edit_3dRobotView_opacity as text
%        str2double(get(hObject,'String')) returns contents of edit_3dRobotView_opacity as a double


% --- Executes during object creation, after setting all properties.
function edit_3dRobotView_opacity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_3dRobotView_opacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in rdiobtn_RobotLinesView.
function rdiobtn_RobotLinesView_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_RobotLinesView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mode_RobotView
mode_RobotView = 0;
set(handles.rdiobtn_RobotCubesView,'Value',0);
axis(handles.axes_robot,[-1 1 -1 1 0 0.75])
global theta1 theta2 d3 theta4
update_end_effector(theta1, theta2, d3, theta4, handles);
% Hint: get(hObject,'Value') returns toggle state of rdiobtn_RobotLinesView


% --- Executes on button press in rdiobtn_RobotCubesView.
function rdiobtn_RobotCubesView_Callback(hObject, eventdata, handles)
% hObject    handle to rdiobtn_RobotCubesView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mode_RobotView
mode_RobotView = 1;
set(handles.rdiobtn_RobotLinesView,'Value',0);
axis(handles.axes_robot,[-1 1 -1 1 0 1.15])
global theta1 theta2 d3 theta4
update_end_effector(theta1, theta2, d3, theta4, handles);
% Hint: get(hObject,'Value') returns toggle state of rdiobtn_RobotCubesView


% --- Executes on button press in cbox_enable_noncentralize_control.
function cbox_enable_noncentralize_control_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_enable_noncentralize_control (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global enable_noncentralize_control
global graph_indication_position;

if (get(handles.cbox_enable_noncentralize_control,'Value') > 0.5)
    enable_noncentralize_control = true;
    set(handles.uipanel_graph_indication,'visible','on','position',graph_indication_position);
else
    enable_noncentralize_control = false;
    set(handles.uipanel_graph_indication,'visible','off','position',graph_indication_position);
end
% Hint: get(hObject,'Value') returns toggle state of cbox_enable_noncentralize_control


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
