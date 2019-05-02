within GIT_thesis_Bram_Stockman;
model RBC_model_ASHRAE_internal_heat_gains_ext2
  extends RBC_model_ASHRAE_internal_heat_gains(
    HeatingCurveOut(table=[-7 + 273,4.85; 3 + 273,2; 9 + 273,0]),
    CoolingCurveOut(table=[12 + 273,0; 15 + 273,-4; 20 + 273,-6]),
    HeatingCurveIn(table=[-8.0 + 273,35 + 273; 15 + 273,24 + 273]));
end RBC_model_ASHRAE_internal_heat_gains_ext2;
