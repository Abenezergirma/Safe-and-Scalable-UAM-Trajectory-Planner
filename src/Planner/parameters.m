%Parameters for FastMDP based trajectory planning 

DEG2RAD=0.0174533; %degree to rad coverting constant 
RAD2DEG = 57.29577951308232; %rad to degree converting constant
MACH = 343; %Speed of sound 
g = 9.81; %gravity
L = 0.9;

%indices for later use in value calculation
RWD  = 1; 
LOCX = 2;
LOCY = 3;
LOCZ = 4;
DISC = 5;
LIM  = 6;

% Berkeley CA
BASE_LAT = 37.8715;
BASE_LON = -122.2730;