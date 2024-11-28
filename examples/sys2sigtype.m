function sigtype = sys2sigtype(sys)

sigtype = NaN(size(sys));
sigtype(sys==gt.C.SYS_GPS) = 0; % GPS
sigtype(sys==gt.C.SYS_GLO) = 1; % GLO
sigtype(sys==gt.C.SYS_GAL) = 2; % GAL
sigtype(sys==gt.C.SYS_QZS) = 3; % QZS
sigtype(sys==gt.C.SYS_CMP) = 4; % BDS