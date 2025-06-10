function sigtype = sys2sigtype(sys, freq)
% Converts satellite system and frequency to signal type index
% Author: Taro Suzuki
arguments
    sys         % Satellite system (gt.C.SYS_XXX)
    freq = "L1" % "L1" or "L2" or "L5"
end

if isscalar(freq)
    freq = repmat(freq,size(sys));
end
sigtype = NaN(size(sys));

sigtype(sys==gt.C.SYS_GPS & freq=="L1") = 0; % GPS L1
sigtype(sys==gt.C.SYS_GLO & freq=="L1") = 1; % GLO L1
sigtype(sys==gt.C.SYS_GAL & freq=="L1") = 2; % GAL L1
sigtype(sys==gt.C.SYS_QZS & freq=="L1") = 3; % QZS L1
sigtype(sys==gt.C.SYS_CMP & freq=="L1") = 4; % BDS L1

sigtype(sys==gt.C.SYS_GPS & freq=="L2") = 5; % GPS L2
sigtype(sys==gt.C.SYS_GLO & freq=="L2") = 6; % GLO L2
sigtype(sys==gt.C.SYS_GAL & freq=="L2") = 7; % GAL L2
sigtype(sys==gt.C.SYS_QZS & freq=="L2") = 8; % QZS L2
sigtype(sys==gt.C.SYS_CMP & freq=="L2") = 9; % BDS L2

sigtype(sys==gt.C.SYS_GPS & freq=="L5") = 10; % GPS L5
sigtype(sys==gt.C.SYS_GLO & freq=="L5") = 11; % GLO L5
sigtype(sys==gt.C.SYS_GAL & freq=="L5") = 12; % GAL L5
sigtype(sys==gt.C.SYS_QZS & freq=="L5") = 13; % QZS L5
sigtype(sys==gt.C.SYS_CMP & freq=="L5") = 14; % BDS L5