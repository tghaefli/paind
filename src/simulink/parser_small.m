%function output_args = parser_small(input_args)

function output_args = parser_small(input_args)


%MAT_PARSER_1 Summary of this function goes here
%   Detailed explanation goes here

global pck_length;
global pck_ctr;

global time_old;
global g_x_old;
global g_y_old;
global g_z_old;

%Debug purpose
pck_ctr = pck_ctr+1;

THRESHOLD = 50;

%Parse package
pck_length = input_args(1,:);
pck_length = pck_length + 256*input_args(2,:);

clc;

time = double(typecast(uint8(input_args(3:10)), 'uint64'))/1e6;
g_x = double(typecast(uint8(input_args(207:210)), 'single'));
g_y = double(typecast(uint8(input_args(211:214)), 'single'));
g_z = double(typecast(uint8(input_args(215:218)), 'single'));    


% crc detector




% crc workaround for legal data
if isnan(time)   time = time_old;       end

if isnan(g_x)         g_x = g_x_old;	end
if (g_x<-THRESHOLD)   g_x = g_x_old;    end
if (g_x>THRESHOLD)    g_x = g_x_old;    end

if isnan(g_y)         g_y = g_y_old;    end
if (g_y<-THRESHOLD)   g_y = g_y_old;    end
if (g_y>THRESHOLD)    g_y = g_y_old;    end

if isnan(g_z)         g_z = g_z_old;    end
if (g_z<-THRESHOLD)   g_z = g_z_old;    end
if (g_z>THRESHOLD)    g_z = g_z_old;    end

g_x_old = g_x;
g_y_old = g_y;
g_z_old = g_z;
   

%output data
output_args = [time ; g_x ; g_y ; g_z ];
            
            