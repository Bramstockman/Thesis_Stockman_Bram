within GIT_thesis_Bram_Stockman;
model RBC_model_ASHRAE_internal_heat_gains_ext8
  extends RBC_model_ASHRAE_internal_heat_gains(
    HeatingCurveIn(table=[-8.0 + 273,35 + 273; 14 + 273,22 + 273]),
    CoolingCurveIn(table=[20 + 273,19 + 273; 23 + 273,14 + 273]),
    fanEmbeddedPipe(m_flow_nominal=1.6),
    heatingCoolingSet1(CoolSet=20 + 273, HeatSet=18.5 + 273),
    CoolingCurveOut(table=[15 + 273,0; 20 + 273,-5; 25 + 273,-7]));
end RBC_model_ASHRAE_internal_heat_gains_ext8;
