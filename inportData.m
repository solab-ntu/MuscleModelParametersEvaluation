function [time, data, text] = inportData(text_line)
[file_input, pathname] = uigetfile({'*.sto', 'OpenSim States Files (*.sto)';...
                                    '*.mot', 'OpenSim Motion Files (*.mot)'}, ...
                                    'Select the initial states file','MultiSelect', 'off');   
temp = importdata(strcat(pathname,file_input)); % import motion data
time = temp.data(:,1);         % time 
data = temp.data(:,2:end);     % the data, e.g. measured coornadites, states and controls
text = temp.textdata(text_line,2:end); % names of the data
end